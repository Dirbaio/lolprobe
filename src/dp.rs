use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use anyhow::bail;
use tracing::trace;

use crate::cmsisdap::commands::transfer::Port;
use crate::probe::Probe;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DpAddress {
    Default,
    Multidrop(u32),
}

pub struct Dp {
    inner: RefCell<DpInner>,
}

struct DpInner {
    probe: Rc<Probe>,
    address: DpAddress,
    last_select: Option<regs::Select>,
}

impl Dp {
    pub fn new(probe: Rc<Probe>, address: DpAddress) -> Self {
        Self {
            inner: RefCell::new(DpInner {
                probe,
                address,
                last_select: None,
            }),
        }
    }

    pub fn read(&self, addr: u8) -> Result<u32, anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.read(addr)
    }
    pub fn write(&self, addr: u8, val: u32) -> Result<(), anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.write(addr, val)
    }

    pub fn read_ap(&self, ap: u8, addr: u8) -> Result<u32, anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.select(|s| {
            s.set_ap_sel(ap);
            s.set_ap_bank_sel(addr >> 4)
        })?;
        let res = inner.probe.read(inner.address, Port::Ap, addr & 0xF)?;
        //let res = inner.probe.read(inner.address, Port::Dp, regs::RDBUFF_ADDR)?;
        Ok(res)
    }

    pub fn write_ap(&self, ap: u8, addr: u8, val: u32) -> Result<(), anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();
        inner.select(|s| {
            s.set_ap_sel(ap);
            s.set_ap_bank_sel(addr >> 4)
        })?;
        inner.probe.write(inner.address, Port::Ap, addr & 0xF, val)?;
        inner.probe.read(inner.address, Port::Dp, regs::RDBUFF_ADDR)?;
        Ok(())
    }

    pub fn powerup(&self) -> Result<(), anyhow::Error> {
        let inner = &mut *self.inner.borrow_mut();

        let mut abort = regs::Abort(0);
        abort.set_dapabort(true);
        abort.set_orunerrclr(true);
        abort.set_wderrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);
        inner.write_abort(abort)?;

        let mut ctrl = regs::Ctrl(0);
        ctrl.set_cdbgpwrupreq(true);
        ctrl.set_csyspwrupreq(true);
        inner.write_ctrl(ctrl)?;

        let start = Instant::now();
        loop {
            let ctrl = inner.read_ctrl()?;
            if ctrl.csyspwrupack() && ctrl.cdbgpwrupack() {
                break;
            }
            if start.elapsed() >= Duration::from_secs(1) {
                bail!("powerup timeout")
            }
        }

        // TODO: Handle JTAG Specific part

        // TODO: Only run the following code when the SWD protocol is used

        // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
        let mut ctrl = regs::Ctrl(0);
        ctrl.set_cdbgpwrupreq(true);
        ctrl.set_csyspwrupreq(true);
        ctrl.set_mask_lane(0b1111);
        inner.write_ctrl(ctrl)?;

        let ctrl = inner.read_ctrl()?;
        if !(ctrl.csyspwrupack() && ctrl.cdbgpwrupack()) {
            bail!("Debug power request failed");
        }

        Ok(())
    }
}

impl DpInner {
    fn read(&mut self, addr: u8) -> Result<u32, anyhow::Error> {
        self.probe.read(self.address, Port::Dp, addr)
    }

    fn write(&mut self, addr: u8, val: u32) -> Result<(), anyhow::Error> {
        self.probe.write(self.address, Port::Dp, addr, val)
    }

    fn read_ctrl(&mut self) -> Result<regs::Ctrl, anyhow::Error> {
        self.read(regs::CTRL_ADDR).map(regs::Ctrl)
    }

    fn write_ctrl(&mut self, val: regs::Ctrl) -> Result<(), anyhow::Error> {
        self.write(regs::CTRL_ADDR, val.0)
    }

    fn write_abort(&mut self, val: regs::Abort) -> Result<(), anyhow::Error> {
        self.write(regs::ABORT_ADDR, val.0)
    }

    fn select(&mut self, f: impl FnOnce(&mut regs::Select)) -> Result<(), anyhow::Error> {
        let mut select = self.last_select.unwrap_or(regs::Select(0));
        f(&mut select);

        if self.last_select == Some(select) {
            return Ok(());
        }

        // clear it, so if selection fails partway we don't consider
        // anything to be selected.
        self.last_select = None;

        trace!("select {:?}", select);
        self.probe.write(self.address, Port::Dp, regs::SELECT_ADDR, select.0)?;

        self.last_select = Some(select);

        Ok(())
    }
}

pub mod regs {
    use bitfield::bitfield;

    pub const ABORT_ADDR: u8 = 0;
    pub const DPIDR_ADDR: u8 = 0;
    pub const CTRL_ADDR: u8 = 4;
    pub const SELECT_ADDR: u8 = 8;
    pub const RDBUFF_ADDR: u8 = 12;

    bitfield! {
        /// ABORT, Abort register (see ADI v5.2 B2.2.1)
        #[derive(Copy, Clone, Default, Eq, PartialEq)]
        pub struct Abort(u32);
        impl Debug;
        /// To clear the CTRL/STAT.STICKYORUN overrun error bit to `0b0`, write `0b1` to this bit.
        pub _, set_orunerrclr: 4;
        /// To clear the CTRL/STAT.WDATAERR write data error bit to `0b0`, write `0b1` to this bit.
        pub _, set_wderrclr: 3;
        /// To clear the CTRL/STAT.STICKYERR sticky error bit to `0b0`, write `0b1` to this bit.
        pub _, set_stkerrclr: 2;
        /// To clear the CTRL/STAT.STICKYCMP sticky compare bit to `0b0`, write `0b1` to this bit. It is IMPLEMENTATION DEFINED whether the CTRL/STAT.STICKYCMP bit is implemented. See MINDP, Minimal DP extension on page B1-40.
        pub _, set_stkcmpclr: 1;
        /// To generate a DAP abort, which aborts the current AP transaction, write `0b1` to this bit. Do this write only if the debugger has received WAIT responses over an extended period. In DPv0, this bit is SBO.
        pub _, set_dapabort: 0;
    }

    bitfield! {
        /// CTRL/STAT, Control/Status register (see ADI v5.2 B2.2.2)
        #[derive(Copy, Clone, Default, Eq, PartialEq)]
        pub struct Ctrl(u32);
        impl Debug;
        /// System powerup acknowledge. Indicates the status of the CSYSPWRUPACK signal. See Power control requirements and operation on page B2-78.
        pub csyspwrupack, _: 31;
        /// System powerup request. This bit controls the CSYSPWRUPREQ signal. See Power control requirements and operation on page B2-78.
        pub csyspwrupreq, set_csyspwrupreq: 30;
        /// Debug powerup acknowledge. Indicates the status of the CDBGPWRUPACK signal. See Power control requirements and operation on page B2-78.
        pub cdbgpwrupack, _: 29;
        /// Debug powerup request. This bit controls the CDBGPWRUPREQ signal. See Power control requirements and operation on page B2-78.
        pub cdbgpwrupreq, set_cdbgpwrupreq: 28;
        /// Debug reset acknowledge. Indicates the status of the CDBGRSTACK signal. See Debug reset control behavior on page B2-81.
        pub cdbgrstack, _: 27;
        /// Debug reset request. This bit controls the CDBGRSTREQ signal. See Debug reset control request on page B2-82.
        ///
        /// After a powerup reset, this bit is `0b0`.
        pub c_dbg_rst_req, set_c_dbg_rst_req: 26;
        /// Error mode. Indicates the reset behavior of the CTRL/STAT.STICKYERR field.
        ///
        /// If true, CTRL/STAT.STICKYERR is cleared when a FAULT response is output.
        ///
        /// After a powerup reset, the value of this field is false.
        ///
        /// This bit only exists on DPv3 (see ADIv6 B2.2.3), it is RES0 on previous DP versions. Its
        /// value should therefore be ignored and written as 0.
        pub errmode, set_errmode: 24;
        /// Transaction counter. See The transaction counter on page B1-43. After a powerup reset, the value of this field is UNKNOWN.
        ///
        /// It is IMPLEMENTATION DEFINED whether this field is implemented.
        ///
        /// TRNCNT is not supported in MINDP configuration. In MINDP configuration, the effect of writing a value other than zero to TRNCNT or TRNMODE is UNPREDICTABLE. See also MINDP, Minimal DP extension on page B1-40.
        pub u16, trn_cnt, set_trn_cnt: 23, 12;
        /// For pushed operations, the DP performs a byte-by-byte comparison of the word that is supplied in an AP write transaction with the current word at the target AP address. The MASKLANE field is used to select the bytes to be included in this comparison. For more information about pushed operations, see Pushed-compare and pushed-verify operations on page B1-44.
        ///
        /// Each of the four bits of the MASKLANE field corresponds to one of the four bytes of the words to be compared. Therefore, each bit is said to control one byte lane of the compare operation.
        ///
        /// Table B2-8 shows how the bits of MASKLANE control the comparison masking.
        pub u8, mask_lane, set_mask_lane: 11, 8;
        /// This bit is set to `0b1` if one of the following Write Data Error occurs:
        ///
        /// - A parity or framing error on the data phase of a write.
        /// - A write that has been accepted by the DP is then discarded without being submitted to the AP. For more information, see Sticky flags and DP error responses on page B1-41.
        ///
        /// Access to and how to clear this field are DATA LINK DEFINED:
        ///
        /// **JTAG-DP, all implementations**
        ///
        /// - Access is reserved, RES0.
        ///
        /// **SW-DP, all implementations, and JTAG-DP, DPv1 and higher**
        ///
        /// - Access is RO/WI.
        /// - To clear WDATAERR to `0b0`, write 0b1 to the ABORT.WDERRCLR field in the ABORT register. A single write of the ABORT register can be used to clear multiple flags if necessary.
        ///
        /// After clearing the WDATAERR flag, you must typically resend the corrupted data. After a powerup reset, WDATAERR is `0b0`.
        pub w_data_err, _ : 7;
        /// This bit is DATA LINK DEFINED
        ///
        /// - On JTAG-DP, the bit is reserved, RES0.\
        /// - On SW-DP, access is RO/WI.
        ///
        /// If the response to the previous AP read or RDBUFF read was OK, the bit is set to 0b1. If the response was not OK, it is cleared to `0b0`.
        ///
        /// This flag always indicates the response to the last AP read access. See Protocol error response on page B4-114.
        ///
        /// After a powerup reset, this bit is `0b0`.
        ///
        /// **Note**
        ///
        /// This field is defined for DPv1 and higher only.
        pub read_ok, _ : 6;
        /// This bit is set to 0b1 if an error is returned by an AP transaction. See Sticky flags and DP error responses on page B1-41.
        ///
        /// Access to and how to clear this field are DATA LINK DEFINED:
        ///
        /// **JTAG-DP, all implementations**
        ///
        /// - Access is R/W1C.
        /// - To clear STICKYERR to 0b0, write 0b1 to it, which signals the DP to clear the flag and set it to 0b0. A single write of the CTRL/STAT register can be used to clear multiple flags if necessary.
        ///
        /// STICKYERR can also be cleared using the ABORT.STKERRCLR field.
        ///
        /// **SW-DP, all implementations, and JTAG-DP, DPv1 and higher**
        ///
        /// - Access is RO/WI.
        /// - To clear STICKYERR to 0b0, write 0b1 to the ABORT.STKERRCLR field in the ABORT register. A single write of the ABORT register can be used to clear multiple flags if necessary.
        ///
        /// After clearing CTRL/STAT.STICKYERR, you must find the location where the error that caused the flag to be set occurred.
        ///
        /// After a powerup reset, this bit is `0b0`.
        pub sticky_err, _: 5;
        /// This bit is set to 0b1 when a mismatch occurs during a pushed-compare operation or a match occurs during a pushed-verify operation. See Pushed-compare and pushed-verify operations on
        /// page B1-44.
        /// It is IMPLEMENTATION DEFINED whether this field is implemented. See MINDP, Minimal DP extension on page B1-40.
        /// Access to and how to clear this field are DATA LINK DEFINED:
        ///
        /// **JTAG-DP, all implementations**
        ///
        /// - Access is R/W1C.
        /// - To clear STICKYCMP to 0b0, write 0b1 to it, which signals the DP to clear the flag and set it to 0b0. A single write of the CTRL/STAT register can be used to clear multiple flags if necessary. STICKYCMP can also be cleared using the ABORT.STKERRCLR field.
        ///
        /// **SW-DP, all implementations, and JTAG-DP, DPv1 and higher**
        ///
        /// - Access is RO/WI.
        /// B2 DP Reference Information B2.2 DP register descriptions
        /// - To clear STICKYCMP to 0b0, write 0b1 to the ABORT.STKCMPCLR field in the ABORT register. A single write of the ABORT register can be used to clear multiple flags if necessary.
        /// After clearing STICKYCMP, you must retrieve the value of the transaction counter to find the location where the error that caused the flag to be set occurred.
        ///
        /// After a powerup reset, this bit is `0b0`.
        pub stick_cmp, _: 4;
        /// This field sets the transfer mode for AP operations.
        /// In normal operation, AP transactions are passed to the AP for processing, as described in _Using the AP to access debug resources_ on page A1-31.
        /// In pushed-verify and pushed-compare operations, the DP compares the value that is supplied in an AP write transaction with the value held in the target AP address. The AP write transaction generates a read access to the debug memory system as described in Pushed-compare and pushed-verify operations on page B1-44.
        ///
        /// TRNMODE can have one of the following values:
        ///
        /// `0b00`: Normal operation.\
        /// `0b01`: Pushed-verify mode.\
        /// `0b10`: Pushed-compare mode.\
        /// `0b11`: Reserved.
        ///
        /// After a powerup reset, the value of this field is UNKNOWN.
        ///
        /// **Note**
        ///
        /// It is IMPLEMENTATION DEFINED whether this field is implemented.
        ///
        /// TRNMODE is not supported in MINDP configuration. In MINDP configuration, the effect of writing a value other than zero to TRNCNT or TRNMODE is UNPREDICTABLE. See also MINDP, Minimal DP extension on page B1-40.
        pub u8, trn_mode, _: 3, 2;
        /// If overrun detection is enabled, this bit is set to 0b1 when an overrun occurs. See bit[0] of this register for details of enabling overrun detection.
        /// Access to and how to clear this field are DATA LINK DEFINED:
        ///
        /// JTAG-DP, all implementations
        /// - Access is R/W1C.
        /// - To clear STICKYORUN to 0b0, write 0b1 to it, which signals the DP to clear the flag and set it to 0b0. A single write of the CTRL/STAT register can be used to clear multiple flags if necessary.
        /// STICKYORUN can also be cleared using the ABORT.STKERRCLR field.
        /// SW-DP, all implementations, and JTAG-DP, DPv1 and higher
        /// - Access is RO/WI.
        /// - To clear STICKYORUN to 0b0, write 0b1 to the ABORT.ORUNERRCLR field in the ABORT register.
        ///
        /// A single write of the ABORT register can be used to clear multiple flags if necessary.
        /// After clearing STICKYORUN, you must find out which DP or AP transaction initiated the overrun that caused the flag to be set, and repeat the transactions for that DP or AP from the transaction pointed to by the transaction counter.
        ///
        /// After a powerup reset, this bit is 0b0.
        pub sticky_orun, _: 1;
        /// This bit can have one of the following values:
        ///
        /// `0b0`: Overrun detection is disabled.\
        /// `0b1`: Overrun detection is enabled.
        ///
        /// For more information about overrun detection, see Sticky flags and DP error responses on page B1-41.
        ///
        /// After a powerup reset, this bit is 0b0.
        pub orun_detect, set_orun_detect: 0;
    }

    bitfield! {
        /// SELECT, AP Select register (see ADI v5.2 B2.2.9)
        #[derive(Copy, Clone, Default, Eq, PartialEq)]
        pub struct Select(u32);
        impl Debug;
        /// Selects the AP with the ID number APSEL. If there is no AP with the ID APSEL, all AP transactions return zero on reads and are ignored on writes. See Register maps, and accesses to reserved addresses on page B2-52.
        /// After a powerup reset, the value of this field is UNKNOWN.
        /// Note
        /// Every Arm Debug Interface implementation must include at least one AP.
        pub u8, ap_sel, set_ap_sel: 31, 24;
        /// Selects the active four-word register bank on the current AP. See Using the AP to access debug resources on page A1-31.
        /// After a powerup reset, the value of this field is UNKNOWN.
        pub u8, ap_bank_sel, set_ap_bank_sel: 7, 4;
        /// Debug Port address bank select.
        /// The behavior of SELECT.DPBANKSEL depends on the DP version, as follows:
        /// DPv0 In DPv0 the SELECT.DPBANKSEL field must be written as zero, otherwise accesses to DP register 0x4 are UNPREDICTABLE.
        /// DPv1 In DPv1 the SELECT.DPBANKSEL field controls which DP register is selected at address 0x4, and Table B2-10 shows the permitted values of this field.
        /// Table B2-10 DPBANKSEL DP register allocation in DPv1
        /// DPBANKSEL DP register at address 0x4
        /// * 0x0 CTRL/STAT
        /// * 0x1 DLCR
        /// All other values of SELECT.DPBANKSEL are reserved. If the field is set to a reserved value, accesses to DP register 0x4 are UNPREDICTABLE.
        /// DPv2 In DPv2 the SELECT.DPBANKSEL field controls which DP register is selected at address 0x4, and Table B2-11 shows the permitted values of this field.
        /// Table B2-11 DPBANKSEL DP register allocation in DPv2 DPBANKSEL DP register at address 0x4
        /// * 0x0 CTRL/STAT
        /// * 0x1 DLCR
        /// * 0x2 TARGETID
        /// * 0x3 DLPIDR
        /// * 0x4 EVENTSTAT
        /// All other values of SELECT.DPBANKSEL are reserved. If the field is set to a reserved value, accesses to DP register 0x4 are RES0.
        /// After a powerup reset, this field is 0x0. Note
        /// Some previous ADI revisions have described DPBANKSEL as a single-bit field called CTRSEL, defined only for SW-DP. From issue B of this document, DPBANKSEL is redefined. The new definition is backwards-compatible.
        pub u8, dp_bank_sel, set_dp_bank_sel: 3, 0;
    }

    bitfield! {
        /// DPIDR, Debug Port Identification register (see ADI v5.2 B2.2.5)
        ///
        /// DPIDR provides information about the Debug Port.
        #[derive(Clone, Eq, PartialEq)]
        pub struct DPIDR(u32);
        impl Debug;
        /// Revision code. The meaning of this field is IMPLEMENTATION DEFINED.
        pub u8, revision, _: 31, 28;
        /// Part Number for the Debug Port. This value is provided by the designer of the Debug Port and must not be changed.
        pub u8, part_no, _: 27, 20;
        /// Minimal Debug Port (MINDP) functions implemented:
        ///
        /// `0b0`: Transaction counter, Pushed-verify, and Pushed-find operations are implemented.\
        /// `0b1`: Transaction counter, Pushed-verify, and Pushed-find operations are not implemented.
        pub min, _: 16;
        /// Version of the Debug Port architecture implemented. Permitted values are:
        ///
        /// `0x0`: Reserved. Implementations of DPv0 do not implement DPIDR.\
        /// `0x1`: DPv1 is implemented.\
        /// `0x2`: DPv2 is implemented.\
        /// `0x3`: DPv3 is implemented.
        ///
        /// All remaining values are reserved.
        pub u8, version, _: 15, 12;
        /// Code that identifies the designer of the DP.
        /// This field indicates the designer of the DP and not the implementer, except where the two are the same.
        /// A JEDEC code takes the following form:
        ///
        /// - A sequence of zero or more numbers, all having the value `0x7F`.
        /// - A following 8-bit number, that is not `0x7F`, and where `bit[7]` is an odd parity bit. For example, Arm Limited is assigned the code `0x7F 0x7F 0x7F 0x7F 0x3B`.
        /// The encoding that is used in the DPIDR is as follows:
        /// - The JEP106 continuation code, DPIDR `bits[11:8]`, is the number of times that `0x7F` appears
        /// before the final number. For example, for Arm Limited this field is `0x4`.
        /// - The JEP106 identification code, IDR `bits[7:1]`, equals `bits[6:0]` of the final number of the
        /// JEDEC code. For example, for Arm Limited this field is `0x3B`.
        pub designer, _: 11, 1;
        /// The JEP106 continuation code (see [`DPIDR::version`]).
        pub u8, jep_cc, _: 11, 8;
        /// The JEP106 ID (see [`DPIDR::version`]).
        pub u8, jep_id, _: 7, 1;
    }

    bitfield! {
        /// TARGETID, Target Identification register (see ADI v5.2 B2.2.10)
        ///
        /// TARGETID provides information about the target when the host is connected to a single device.
        #[derive(Copy, Clone, Default, Eq, PartialEq)]
        pub struct TARGETID(u32);
        impl Debug;
        /// Target revision.
        pub u8, trevision, _: 31, 28;
        /// IMPLEMENTATION DEFINED. The value is assigned by the designer of the part. The value must be unique to the part.
        pub u16, tpartno, _: 27, 12;
        /// IMPLEMENTATION DEFINED.
        ///
        /// This field indicates the designer of the part and not the implementer, except where the two are the same.
        /// Designers must insert their JEDEC-assigned code here.
        ///
        /// **Note**
        ///
        /// The Arm JEP106 value is not shown for the TDESIGNER field. Arm might design a DP containing the TARGETID register, but typically, the designer of the part, referenced in the TPARTNO field, is another designer who creates a part around the licensed Arm IP. If the designer of the part is Arm, then the value of this field is `0x23B`.
        ///
        /// A JEP106 code takes the following form:
        ///
        /// - A sequence of zero or more numbers, all having the value `0x7F`.
        /// - A following 8-bit number, that is not `0x7F`, and where `bit[7]` is an odd parity bit. For example, Arm Limited is assigned the code `0x7F 0x7F 0x7F 0x7F 0x3B`.
        /// The encoding that is used in TARGETID is as follows:
        /// - The JEP106 continuation code, TARGETID `bits[11:8]`, is the number of times that `0x7F`
        /// appears before the final number. For example, for Arm Limited this field is `0x4`.
        /// - The JEP106 identification code, TARGETID `bits[7:1]`, equals `bits[6:0]` of the final number of the JEDEC code.
        pub u16, tdesigner, _: 11, 1;
    }
}
