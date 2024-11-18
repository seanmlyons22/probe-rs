//! Sequences for cc23xx_cc27xx devices
use std::ops::DerefMut;
use std::sync::Arc;
use std::thread;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crate::architecture::arm::ap::AccessPortType;
use crate::architecture::arm::armv6m::{Aircr, Demcr, Dhcsr};
use crate::architecture::arm::communication_interface::{ArmCommunicationInterface, Initialized};
use crate::architecture::arm::core::cortex_m;
use crate::architecture::arm::dp::{Abort, Ctrl, DebugPortError, DpAccess, Select};
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{cortex_m_core_start, ArmDebugSequence};
use crate::architecture::arm::ArmProbeInterface;
use crate::architecture::arm::DapAccess;
use crate::architecture::arm::{ArmError, DpAddress, FullyQualifiedApAddress};
use crate::MemoryMappedRegister;
use probe_rs_target::CoreType;

static BOOT_LOOP: AtomicBool = AtomicBool::new(false);

fn is_in_boot_loop() -> bool {
    BOOT_LOOP.load(Ordering::SeqCst)
}

/// Marker struct indicating initialization sequencing for cc23xx_cc27xx family parts.
#[derive(Debug)]
pub struct CC23xxCC27xx {
    // Chip name
    _name: String,
}

impl CC23xxCC27xx {
    /// Create the sequencer for the cc13xx_cc26xx family of parts.
    pub fn create(name: String) -> Arc<Self> {
        Arc::new(Self { _name: name })
    }
    fn saci_command(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        command: u32,
    ) -> Result<(), ArmError> {
        let sec_ap = &FullyQualifiedApAddress::v1_with_default_dp(2);

        const TX_DATA_ADDR: u8 = 0;
        const TX_CTRL_ADDR: u8 = 4;

        let mut tx_ctrl = interface.read_raw_ap_register(sec_ap, TX_CTRL_ADDR)?;

        // Wait for tx_ctrl to be ready
        while (tx_ctrl & 0x00000001) != 0 {
            tx_ctrl = interface.read_raw_ap_register(sec_ap, TX_CTRL_ADDR)?;
        }

        // Set Cmd Start
        interface.write_raw_ap_register(sec_ap, TX_CTRL_ADDR, 0x02)?;

        // Write parameter word to txd
        interface.write_raw_ap_register(sec_ap, TX_DATA_ADDR, command)?;

        // Wait for tx_ctrl to be ready
        tx_ctrl = interface.read_raw_ap_register(sec_ap, TX_CTRL_ADDR)?;
        while (tx_ctrl & 0x00000001) != 0 {
            tx_ctrl = interface.read_raw_ap_register(sec_ap, TX_CTRL_ADDR)?;
        }

        Ok(())
    }
}

impl ArmDebugSequence for CC23xxCC27xx {
    fn reset_system(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // Check if the previous code requested a halt before reset
        let demcr = Demcr(probe.read_word_32(Demcr::get_mmio_address())?);

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);

        probe.write_word_32(Aircr::get_mmio_address(), aircr.into())?;

        // Re-initializing the core(s) is on us.
        let ap = probe.ap().ap_address().clone();
        let interface = probe.get_arm_communication_interface()?;
        interface.reinitialize()?;
        self.debug_core_start(interface, &ap, core_type, debug_base, None)?;

        // Halt the CPU
        if demcr.vc_corereset() {
            let mut value = Dhcsr(0);
            value.set_c_halt(true);
            value.set_c_debugen(true);
            value.enable_write();

            probe.write_word_32(Dhcsr::get_mmio_address(), value.into())?;
        }

        Ok(())
    }

    fn debug_port_start(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        // TODO:
        // COPY_PASTE BEGIN
        let mut abort = Abort(0);
        abort.set_dapabort(true);
        abort.set_orunerrclr(true);
        abort.set_wderrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);
        interface.write_dp_register(dp, abort)?;

        interface.write_dp_register(dp, Select(0))?;

        let ctrl = interface.read_dp_register::<Ctrl>(dp)?;

        let powered_down = !(ctrl.csyspwrupack() && ctrl.cdbgpwrupack());

        if powered_down {
            tracing::info!("Debug port {dp:x?} is powered down, powering up");
            let mut ctrl = Ctrl(0);
            ctrl.set_cdbgpwrupreq(true);
            ctrl.set_csyspwrupreq(true);
            interface.write_dp_register(dp, ctrl)?;

            let start = Instant::now();
            loop {
                let ctrl = interface.read_dp_register::<Ctrl>(dp)?;
                if ctrl.csyspwrupack() && ctrl.cdbgpwrupack() {
                    break;
                }
                if start.elapsed() >= Duration::from_secs(1) {
                    return Err(ArmError::Timeout);
                }
            }

            // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
            let mut ctrl = Ctrl(0);
            ctrl.set_cdbgpwrupreq(true);
            ctrl.set_csyspwrupreq(true);
            ctrl.set_mask_lane(0b1111);
            interface.write_dp_register(dp, ctrl)?;

            let ctrl_reg: Ctrl = interface.read_dp_register(dp)?;
            if !(ctrl_reg.csyspwrupack() && ctrl_reg.cdbgpwrupack()) {
                tracing::error!("Debug power request failed");
                return Err(DebugPortError::TargetPowerUpFailed.into());
            }

            // According to CMSIS docs, here's where we would clear errors
            // in ABORT, but we do that above instead.
        }
        // TODO: COPY_PASTE END

        // This code is unique to the cc23xx_cc27xx family
        // First connect to the config AP to read the device status register
        // This will tell us the state of the boot rom and if SACI is enabled
        let cfg_ap = &FullyQualifiedApAddress::v1_with_default_dp(1);

        let mut device_status = interface.read_raw_ap_register(cfg_ap, 0x0C).unwrap();

        // Check if bit 24 of the device status register is cleared
        // If it is, we need to exit SACI to enable the AHB-AP to be accessed
        if device_status & (1 << 24) == 0 {
            // Send the SACI command to exit SACI
            self.saci_command(interface, 0x07)?;

            // Read the device status register again to check if boot is completed
            device_status = interface.read_raw_ap_register(cfg_ap, 0x0C).unwrap();

            // Boot status tells us the state of the boot rom
            let boot_status = (device_status >> 8) & 0xFF;

            // Check if the boot rom is waiting for a debugger to attach
            match boot_status {
                0x38 | 0x81 | 0xC1 => {
                    tracing::info!("BOOT_WAITLOOP_DBGPROBE");
                    BOOT_LOOP.store(true, Ordering::SeqCst);
                }
                _ => tracing::info!("Not waiting on boot status"),
            }
        }

        Ok(())
    }

    fn debug_core_start(
        &self,
        interface: &mut dyn ArmProbeInterface,
        core_ap: &FullyQualifiedApAddress,
        _core_type: CoreType,
        _debug_base: Option<u64>,
        _cti_base: Option<u64>,
    ) -> Result<(), ArmError> {
        if is_in_boot_loop() {
            // Step 1: Halt the CPU
            let mut value = Dhcsr(0);
            value.set_c_halt(true);
            value.set_c_debugen(true);
            value.enable_write();

            let mut memory =
                interface.memory_interface(&FullyQualifiedApAddress::v1_with_default_dp(0))?;
            memory.write_word_32(Dhcsr::get_mmio_address(), value.into())?;
            // Step 1.1: Wait for CPU to halt. TODO, this can be better/more efficient
            thread::sleep(Duration::from_millis(1));

            // Step 2: Write R3 to 0 to exit the boot loop
            cortex_m::write_core_reg(memory.deref_mut(), crate::RegisterId(3), 0x00000000)?;

            // Step 3: Clear the BOOT_LOOP flag
            BOOT_LOOP.store(false, Ordering::SeqCst);
        }

        // Step 4: Start the core like normal
        let mut core = interface.memory_interface(core_ap)?;
        cortex_m_core_start(&mut *core)
    }
}
