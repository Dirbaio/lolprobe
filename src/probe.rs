use std::cell::RefCell;

use tracing::trace;

use crate::cmsisdap::commands::transfer::Port;
use crate::cmsisdap::CmsisDap;
use crate::dp::DpAddress;

pub struct Probe {
    pub inner: RefCell<ProbeInner>,
}

pub struct ProbeInner {
    pub dev: CmsisDap,
    selected_dp: Option<DpAddress>,
}

impl Probe {
    pub fn new(dev: CmsisDap) -> Self {
        Self {
            inner: RefCell::new(ProbeInner { dev, selected_dp: None }),
        }
    }

    pub fn read(&self, dp: DpAddress, port: Port, addr: u8) -> Result<u32, anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.select_dp(dp)?;
        inner.dev.read(port, addr)
    }

    pub fn write(&self, dp: DpAddress, port: Port, addr: u8, val: u32) -> Result<(), anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.select_dp(dp)?;
        inner.dev.write(port, addr, val)
    }
}

impl ProbeInner {
    fn select_dp(&mut self, dp: DpAddress) -> Result<(), anyhow::Error> {
        if self.selected_dp == Some(dp) {
            return Ok(());
        }

        // clear it, so if selection fails partway we don't consider
        // anything to be selected.
        self.selected_dp = None;

        self.swd_line_reset(0)?;

        match dp {
            DpAddress::Multidrop(_) => {
                // Select Dormant State (from JTAG)
                tracing::debug!("Select Dormant State (from JTAG)");
                self.dev.swj_sequence(31, 0x33BBBBBA)?;

                // Leave dormant state
                self.alert_sequence()?;

                // 4 cycles SWDIO/TMS LOW + 8-Bit SWD Activation Code (0x1A)
                self.dev.swj_sequence(12, 0x1A0)?;
            }

            DpAddress::Default => {
                // Execute SWJ-DP Switch Sequence JTAG to SWD (0xE79E).
                // Change if SWJ-DP uses deprecated switch code (0xEDB6).
                self.dev.swj_sequence(16, 0xE79E)?;

                // > 50 cycles SWDIO/TMS High, at least 2 idle cycles (SWDIO/TMS Low).
                // -> done in debug_port_connect
            }
        }

        self.swd_line_reset(3)?;

        // If multidrop is used, we now have to select a target
        if let DpAddress::Multidrop(targetsel) = dp {
            // Deselect other debug ports first?

            tracing::debug!("Writing targetsel {:#x}", targetsel);
            // TARGETSEL write.
            // The TARGETSEL write is not ACKed by design. We can't use a normal register write
            // because many probes don't even send the data phase when NAK.
            let parity = targetsel.count_ones() % 2;
            let data = (parity as u64) << 45 | (targetsel as u64) << 13 | 0x1f99;

            // Should this be a swd_sequence?
            // Technically we shouldn't drive SWDIO all the time when sending a request.
            self.dev.swj_sequence(6 * 8, data)?;
        }

        let dpidr = self.dev.read(Port::Dp, 0)?;
        tracing::debug!("dpidr {:#x}", dpidr);

        self.selected_dp = Some(dp);

        Ok(())
    }

    fn alert_sequence(&mut self) -> Result<(), anyhow::Error> {
        tracing::trace!("Sending Selection Alert sequence");

        // Ensure target is not in the middle of detecting a selection alert
        self.dev.swj_sequence(8, 0xFF)?;

        // Alert Sequence Bits  0.. 63
        self.dev.swj_sequence(64, 0x86852D956209F392)?;

        // Alert Sequence Bits 64..127
        self.dev.swj_sequence(64, 0x19BC0EA2E3DDAFE9)?;

        Ok(())
    }

    /// Perform a SWD line reset (SWDIO high for 50 clock cycles)
    ///
    /// After the line reset, SWDIO will be kept low for `swdio_low_cycles` cycles.
    fn swd_line_reset(&mut self, swdio_low_cycles: u8) -> Result<(), anyhow::Error> {
        assert!(swdio_low_cycles + 51 <= 64);

        tracing::debug!("Performing SWD line reset");
        self.dev.swj_sequence(51 + swdio_low_cycles, 0x0007_FFFF_FFFF_FFFF)?;

        Ok(())
    }
}
