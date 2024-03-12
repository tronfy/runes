use std::collections::HashMap;

use crate::{
    bus::Bus,
    instructions::{create_lookup_table, AddressingMode, Instruction, OpCode},
};

type AM = AddressingMode;
type OP = OpCode;

/// emulated 6502/2A03 CPU
pub struct CPU {
    pub bus: Bus,

    // instruction lookup vector
    instructions: Vec<Instruction>,

    pub a: u8, // accumulator
    pub x: u8,
    pub y: u8,
    pub pc: u16, // program counter
    pub sp: u8,  // stack pointer
    pub status: u8,

    pub fetched: u8,
    pub addr_abs: u16,
    pub addr_rel: u16,
    pub opcode: u8,
    pub cycles_left: u8,
}

pub enum Flag {
    C = 1 << 0, // carry bit
    Z = 1 << 1, // zero
    I = 1 << 2, // disable interrupts
    D = 1 << 3, // decimal mode (unused in 2A03)
    B = 1 << 4, // break
    U = 1 << 5, // unused
    V = 1 << 6, // overflow
    N = 1 << 7, // negative
}

impl CPU {
    pub fn new(bus: Bus) -> CPU {
        CPU {
            bus,
            instructions: create_lookup_table(),
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0,
            status: 0,
            fetched: 0,
            addr_abs: 0,
            addr_rel: 0,
            opcode: 0,
            cycles_left: 0,
        }
    }

    fn read(&self, address: u16) -> u8 {
        self.bus.read(address)
    }

    fn write(&mut self, address: u16, data: u8) {
        self.bus.write(address, data);
    }

    fn set_flag(&mut self, flag: Flag, value: bool) {
        if value {
            self.status |= flag as u8;
        } else {
            self.status &= !(flag as u8);
        }
    }

    pub fn get_flag(&self, flag: Flag) -> bool {
        (self.status & (flag as u8)) != 0
    }

    pub fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0xFD;
        self.status = 0x00 | Flag::U as u8;

        self.addr_abs = 0xFFFC;
        let lo = self.read(self.addr_abs) as u16;
        let hi = self.read(self.addr_abs + 1) as u16;
        self.pc = (hi << 8) | lo;

        self.addr_rel = 0;
        self.addr_abs = 0;
        self.fetched = 0;

        self.cycles_left = 8;
    }

    fn _interrupt(&mut self, addr: u16, cycles: u8) {
        self.write(0x0100 + self.sp as u16, (self.pc >> 8) as u8);
        self.sp -= 1;
        self.write(0x0100 + self.sp as u16, (self.pc & 0x00FF) as u8);
        self.sp -= 1;

        self.set_flag(Flag::B, false);
        self.set_flag(Flag::U, true);
        self.set_flag(Flag::I, true);
        self.write(0x0100 + self.sp as u16, self.status);
        self.sp -= 1;

        self.addr_abs = addr;
        let lo = self.read(self.addr_abs) as u16;
        let hi = self.read(self.addr_abs + 1) as u16;
        self.pc = (hi << 8) | lo;

        self.cycles_left = cycles;
    }

    fn irq(&mut self) {
        if !self.get_flag(Flag::I) {
            self._interrupt(0xFFFE, 7);
        }
    }

    fn nmi(&mut self) {
        self._interrupt(0xFFFA, 8);
    }

    fn set_addressing_mode(&mut self, mode: AddressingMode) -> u8 {
        match mode {
            AM::IMP => self.am_imp(),
            AM::IMM => self.am_imm(),
            AM::ZP0 => self.am_zp0(),
            AM::ZPX => self.am_zpx(),
            AM::ZPY => self.am_zpy(),
            AM::ABS => self.am_abs(),
            AM::ABX => self.am_abx(),
            AM::ABY => self.am_aby(),
            AM::IND => self.am_ind(),
            AM::IZX => self.am_izx(),
            AM::IZY => self.am_izy(),
            AM::REL => self.am_rel(),
        }
    }

    fn exec_opcode(&mut self, code: OpCode) -> u8 {
        match code {
            OP::ADC => self.op_adc(),
            OP::AND => self.op_and(),
            OP::ASL => self.op_asl(),
            OP::BCC => self.op_bcc(),
            OP::BCS => self.op_bcs(),
            OP::BEQ => self.op_beq(),
            OP::BIT => self.op_bit(),
            OP::BMI => self.op_bmi(),
            OP::BNE => self.op_bne(),
            OP::BPL => self.op_bpl(),
            OP::BRK => self.op_brk(),
            OP::BVC => self.op_bvc(),
            OP::BVS => self.op_bvs(),
            OP::CLC => self.op_clc(),
            OP::CLD => self.op_cld(),
            OP::CLI => self.op_cli(),
            OP::CLV => self.op_clv(),
            OP::CMP => self.op_cmp(),
            OP::CPX => self.op_cpx(),
            OP::CPY => self.op_cpy(),
            OP::DEC => self.op_dec(),
            OP::DEX => self.op_dex(),
            OP::DEY => self.op_dey(),
            OP::EOR => self.op_eor(),
            OP::INC => self.op_inc(),
            OP::INX => self.op_inx(),
            OP::INY => self.op_iny(),
            OP::JMP => self.op_jmp(),
            OP::JSR => self.op_jsr(),
            OP::LDA => self.op_lda(),
            OP::LDX => self.op_ldx(),
            OP::LDY => self.op_ldy(),
            OP::LSR => self.op_lsr(),
            OP::NOP => self.op_nop(),
            OP::ORA => self.op_ora(),
            OP::PHA => self.op_pha(),
            OP::PHP => self.op_php(),
            OP::PLA => self.op_pla(),
            OP::PLP => self.op_plp(),
            OP::ROL => self.op_rol(),
            OP::ROR => self.op_ror(),
            OP::RTI => self.op_rti(),
            OP::RTS => self.op_rts(),
            OP::SBC => self.op_sbc(),
            OP::SEC => self.op_sec(),
            OP::SED => self.op_sed(),
            OP::SEI => self.op_sei(),
            OP::STA => self.op_sta(),
            OP::STX => self.op_stx(),
            OP::STY => self.op_sty(),
            OP::TAX => self.op_tax(),
            OP::TAY => self.op_tay(),
            OP::TSX => self.op_tsx(),
            OP::TXA => self.op_txa(),
            OP::TXS => self.op_txs(),
            OP::TYA => self.op_tya(),

            OpCode::XXX => panic!("Illegal opcode"),
        }
    }

    pub fn clock(&mut self) {
        if self.cycles_left == 0 {
            self.opcode = self.read(self.pc);
            self.pc += 1;

            let instruction = self.instructions[self.opcode as usize];
            self.cycles_left = instruction.cycles;

            let am_extra_cycle = self.set_addressing_mode(instruction.addressing_mode);
            let op_extra_cycle = self.exec_opcode(instruction.opcode);

            self.cycles_left += am_extra_cycle & op_extra_cycle;
        }

        self.cycles_left -= 1;
    }

    pub fn complete(&self) -> bool {
        self.cycles_left == 0
    }

    pub fn disassemble(&self, start: u16, end: u16) -> HashMap<u16, String> {
        let mut map = HashMap::new();
        let mut pc = start;

        while pc <= end {
            let line_addr = pc;
            let mut s_inst = format!("{:04X}: ", pc);
            let opcode = self.read(pc);
            let instruction = self.instructions[opcode as usize];
            pc += 1;

            match instruction.addressing_mode {
                AM::IMP => s_inst.push_str(&format!("{:?}", instruction.opcode)),
                AM::IMM => {
                    s_inst.push_str(&format!("{:?} #{:02X}", instruction.opcode, self.read(pc)));
                    pc += 1;
                }
                AM::ZP0 => {
                    s_inst.push_str(&format!("{:?} {:02X}", instruction.opcode, self.read(pc)));
                    pc += 1;
                }
                AM::ZPX => {
                    s_inst.push_str(&format!("{:?} {:02X},X", instruction.opcode, self.read(pc)));
                    pc += 1;
                }
                AM::ZPY => {
                    s_inst.push_str(&format!("{:?} {:02X},Y", instruction.opcode, self.read(pc)));
                    pc += 1;
                }
                AM::IZX => {
                    s_inst.push_str(&format!(
                        "{:?} ({:02X},X)",
                        instruction.opcode,
                        self.read(pc)
                    ));
                    pc += 1;
                }
                AM::IZY => {
                    s_inst.push_str(&format!(
                        "{:?} ({:02X}),Y",
                        instruction.opcode,
                        self.read(pc)
                    ));
                    pc += 1;
                }
                AM::ABS => {
                    let lo = self.read(pc) as u16;
                    pc += 1;
                    let hi = self.read(pc) as u16;
                    pc += 1;
                    let addr = (hi << 8) | lo;
                    s_inst.push_str(&format!("{:?} {:04X}", instruction.opcode, addr));
                }
                AM::ABX => {
                    let lo = self.read(pc) as u16;
                    pc += 1;
                    let hi = self.read(pc) as u16;
                    pc += 1;
                    let addr = (hi << 8) | lo;
                    s_inst.push_str(&format!("{:?} {:04X},X", instruction.opcode, addr));
                }
                AM::ABY => {
                    let lo = self.read(pc) as u16;
                    pc += 1;
                    let hi = self.read(pc) as u16;
                    pc += 1;
                    let addr = (hi << 8) | lo;
                    s_inst.push_str(&format!("{:?} {:04X},Y", instruction.opcode, addr));
                }
                AM::IND => {
                    let lo = self.read(pc) as u16;
                    pc += 1;
                    let hi = self.read(pc) as u16;
                    pc += 1;
                    let addr = (hi << 8) | lo;
                    s_inst.push_str(&format!("{:?} ({:04X})", instruction.opcode, addr));
                }
                AM::REL => {
                    let offset = self.read(pc) as u16;
                    pc += 1;
                    let addr = pc.wrapping_add(offset);
                    s_inst.push_str(&format!("{:?} {:04X}", instruction.opcode, addr));
                }
            }

            map.insert(line_addr, s_inst);
        }

        map
    }

    // === addressing modes ===

    // implied
    fn am_imp(&mut self) -> u8 {
        self.fetched = self.a;
        0
    }

    // immediate
    fn am_imm(&mut self) -> u8 {
        self.addr_abs = self.pc;
        self.pc += 1;
        0
    }

    fn _am_zp_offset(&mut self, offset: u8) -> u8 {
        self.addr_abs = self.read(self.pc) as u16 + offset as u16;
        self.pc += 1;
        self.addr_abs &= 0x00FF;
        0
    }

    // zero page
    fn am_zp0(&mut self) -> u8 {
        self._am_zp_offset(0)
    }

    // zero page with x offset
    fn am_zpx(&mut self) -> u8 {
        self._am_zp_offset(self.x)
    }

    // zero page with y offset
    fn am_zpy(&mut self) -> u8 {
        self._am_zp_offset(self.y)
    }

    fn _am_abs_offset(&mut self, offset: u16) -> u8 {
        let lo = self.read(self.pc) as u16;
        self.pc += 1;
        let hi = self.read(self.pc) as u16;
        self.pc += 1;

        self.addr_abs = (hi << 8) | lo;
        self.addr_abs += offset;

        if offset != 0 && (self.addr_abs & 0xFF00) != (hi << 8) {
            1
        } else {
            0
        }
    }

    // absolute
    fn am_abs(&mut self) -> u8 {
        self._am_abs_offset(0)
    }

    // absolute with x offset
    fn am_abx(&mut self) -> u8 {
        self._am_abs_offset(self.x as u16)
    }

    // absolute with y offset
    fn am_aby(&mut self) -> u8 {
        self._am_abs_offset(self.y as u16)
    }

    // indirect
    fn am_ind(&mut self) -> u8 {
        let ptr_lo = self.read(self.pc) as u16;
        self.pc += 1;
        let ptr_hi = self.read(self.pc) as u16;
        self.pc += 1;

        let ptr = (ptr_hi << 8) | ptr_lo;

        // 6502 bug: An indirect JMP (xxFF) will fail because the MSB will be fetched from address xx00 instead of page xx+1.
        if ptr_lo == 0x00FF {
            // simulate bug
            self.addr_abs = ((self.read(ptr & 0xFF00) as u16) << 8) | self.read(ptr) as u16;
        } else {
            // behave normally
            self.addr_abs = ((self.read(ptr + 1) as u16) << 8) | self.read(ptr) as u16;
        }

        0
    }

    // indirect zero page with x offset
    fn am_izx(&mut self) -> u8 {
        let t = self.read(self.pc) as u16;
        self.pc += 1;

        let lo = self.read((t + self.x as u16) & 0x00FF) as u16;
        let hi = self.read((t + self.x as u16 + 1) & 0x00FF) as u16;

        self.addr_abs = (hi << 8) | lo;

        0
    }

    // indirect zero page with y offset
    fn am_izy(&mut self) -> u8 {
        let t = self.read(self.pc) as u16;
        self.pc += 1;

        let lo = self.read(t & 0x00FF) as u16;
        let hi = self.read((t + 1) & 0x00FF) as u16;

        self.addr_abs = (hi << 8) | lo;
        self.addr_abs += self.y as u16;

        if (self.addr_abs & 0xFF00) != (hi << 8) {
            1
        } else {
            0
        }
    }

    // relative
    fn am_rel(&mut self) -> u8 {
        self.addr_rel = self.read(self.pc) as u16;
        self.pc += 1;

        if (self.addr_rel & 0x80) != 0 {
            self.addr_rel |= 0xFF00;
        }

        0
    }

    // === instructions ===

    fn fetch(&mut self) -> u8 {
        match self.instructions[self.opcode as usize].addressing_mode {
            AM::IMP => {}
            _ => self.fetched = self.read(self.addr_abs),
        };

        self.fetched
    }

    fn op_and(&mut self) -> u8 {
        self.fetch();
        self.a &= self.fetched;
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        1
    }

    fn _op_branch_on_flag(&mut self, flag: Flag, value: bool) -> u8 {
        if self.get_flag(flag) == value {
            self.cycles_left += 1;
            self.addr_abs = self.pc.wrapping_add(self.addr_rel);

            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00) {
                self.cycles_left += 1;
            }

            self.pc = self.addr_abs;
        }
        0
    }

    fn op_bcs(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::C, true)
    }

    fn op_bcc(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::C, false)
    }

    fn op_beq(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::Z, true)
    }

    fn op_bne(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::Z, false)
    }

    fn op_bmi(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::N, true)
    }

    fn op_bpl(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::N, false)
    }

    fn op_bvs(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::V, true)
    }

    fn op_bvc(&mut self) -> u8 {
        self._op_branch_on_flag(Flag::V, false)
    }

    fn _op_set_flag(&mut self, flag: Flag, value: bool) -> u8 {
        self.set_flag(flag, value);
        0
    }

    fn op_clc(&mut self) -> u8 {
        self._op_set_flag(Flag::C, false)
    }

    fn op_cld(&mut self) -> u8 {
        self._op_set_flag(Flag::D, false)
    }

    fn op_cli(&mut self) -> u8 {
        self._op_set_flag(Flag::I, false)
    }

    fn op_clv(&mut self) -> u8 {
        self._op_set_flag(Flag::V, false)
    }

    fn op_adc(&mut self) -> u8 {
        self.fetch();

        // add in 16-bit space so we can detect overflow
        let temp: u16 = self.a as u16 + self.fetched as u16 + self.get_flag(Flag::C) as u16;

        self.set_flag(Flag::C, temp > 255);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);
        self.set_flag(
            Flag::V,
            (!((self.a ^ self.fetched) & 0x80) != 0) && ((self.a ^ temp as u8) & 0x80) != 0,
        );

        self.a = (temp & 0x00FF) as u8;
        1
    }

    fn op_sbc(&mut self) -> u8 {
        self.fetch();

        let value = self.fetched as u16 ^ 0x00FF;

        // add (!) in 16-bit space so we can detect overflow
        let temp: u16 = self.a as u16 + value + self.get_flag(Flag::C) as u16;

        self.set_flag(Flag::C, temp & 0xFF00 != 0);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);
        self.set_flag(
            Flag::V,
            ((temp ^ self.a as u16) & (temp ^ value) & 0x0080) != 0,
        );

        self.a = temp as u8;
        1
    }

    fn op_pha(&mut self) -> u8 {
        self.write(0x0100 + self.sp as u16, self.a);
        self.sp -= 1;
        0
    }

    fn op_pla(&mut self) -> u8 {
        self.sp += 1;
        self.a = self.read(0x0100 + self.sp as u16);
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        0
    }

    fn op_rti(&mut self) -> u8 {
        self.sp += 1;
        self.status = self.read(0x0100 + self.sp as u16);
        self.status &= !(Flag::B as u8);
        self.status &= !(Flag::U as u8);

        self.sp += 1;
        self.pc = self.read(0x0100 + self.sp as u16) as u16;
        self.sp += 1;
        self.pc |= (self.read(0x0100 + self.sp as u16) as u16) << 8;
        0
    }

    fn op_asl(&mut self) -> u8 {
        self.fetch();
        let temp = (self.fetched as u16) << 1;
        self.set_flag(Flag::C, (temp & 0xFF00) > 0);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);

        match self.instructions[self.opcode as usize].addressing_mode {
            AM::IMP => self.a = (temp & 0x00FF) as u8,
            _ => self.write(self.addr_abs, (temp & 0x00FF) as u8),
        }
        0
    }

    fn op_bit(&mut self) -> u8 {
        self.fetch();
        let temp = self.a & self.fetched;
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (self.fetched & (1 << 7)) != 0);
        self.set_flag(Flag::V, (self.fetched & (1 << 6)) != 0);
        0
    }

    fn op_brk(&mut self) -> u8 {
        self.pc += 1;

        self.set_flag(Flag::I, true);
        self.write(0x0100 + self.sp as u16, (self.pc >> 8) as u8);
        self.sp -= 1;
        self.write(0x0100 + self.sp as u16, (self.pc & 0x00FF) as u8);
        self.sp -= 1;

        self.set_flag(Flag::B, true);
        self.write(0x0100 + self.sp as u16, self.status);
        self.sp -= 1;
        self.set_flag(Flag::B, false);

        self.pc = (self.read(0xFFFE) as u16) | ((self.read(0xFFFF) as u16) << 8);
        0
    }

    fn _op_cmp(&mut self, reg: u8) -> u8 {
        self.fetch();
        let temp = reg as u16 - self.fetched as u16;
        self.set_flag(Flag::C, reg >= self.fetched);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);
        1
    }

    fn op_cmp(&mut self) -> u8 {
        self._op_cmp(self.a)
    }

    fn op_cpx(&mut self) -> u8 {
        self._op_cmp(self.x)
    }

    fn op_cpy(&mut self) -> u8 {
        self._op_cmp(self.y)
    }

    fn op_dec(&mut self) -> u8 {
        let mut fetched = self.fetch();
        fetched = fetched.wrapping_sub(1);
        self.set_flag(Flag::Z, fetched == 0);
        self.set_flag(Flag::N, (fetched & 0x80) != 0);
        self.write(self.addr_abs, fetched);
        0
    }

    fn op_dex(&mut self) -> u8 {
        self.x = self.x.wrapping_sub(1);
        self.set_flag(Flag::Z, self.x == 0);
        self.set_flag(Flag::N, (self.x & 0x80) != 0);
        0
    }

    fn op_dey(&mut self) -> u8 {
        self.y = self.y.wrapping_sub(1);
        self.set_flag(Flag::Z, self.y == 0);
        self.set_flag(Flag::N, (self.y & 0x80) != 0);
        0
    }

    fn op_eor(&mut self) -> u8 {
        self.fetch();
        self.a ^= self.fetched;
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        1
    }

    fn _op_inc(&mut self, reg: &mut u8) -> u8 {
        *reg = reg.wrapping_add(1);
        self.set_flag(Flag::Z, *reg == 0);
        self.set_flag(Flag::N, (*reg & 0x80) != 0);
        0
    }

    fn op_inc(&mut self) -> u8 {
        let mut fetched = self.fetch();
        self._op_inc(&mut fetched)
    }

    fn op_inx(&mut self) -> u8 {
        let mut temp_x = self.x;
        self._op_inc(&mut temp_x);
        self.x = temp_x;
        0
    }

    fn op_iny(&mut self) -> u8 {
        let mut temp_y = self.y;
        self._op_inc(&mut temp_y);
        self.y = temp_y;
        0
    }

    fn op_jmp(&mut self) -> u8 {
        self.pc = self.addr_abs;
        0
    }

    fn op_jsr(&mut self) -> u8 {
        self.pc -= 1;
        self.write(0x0100 + self.sp as u16, (self.pc >> 8) as u8);
        self.sp -= 1;
        self.write(0x0100 + self.sp as u16, (self.pc & 0x00FF) as u8);
        self.sp -= 1;

        self.pc = self.addr_abs;
        0
    }

    fn op_lda(&mut self) -> u8 {
        self.a = self.fetch();
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        1
    }

    fn op_ldx(&mut self) -> u8 {
        self.x = self.fetch();
        self.set_flag(Flag::Z, self.x == 0);
        self.set_flag(Flag::N, (self.x & 0x80) != 0);
        1
    }

    fn op_ldy(&mut self) -> u8 {
        self.y = self.fetch();
        self.set_flag(Flag::Z, self.y == 0);
        self.set_flag(Flag::N, (self.y & 0x80) != 0);
        1
    }

    fn op_lsr(&mut self) -> u8 {
        self.fetch();
        self.set_flag(Flag::C, (self.fetched & 0x0001) != 0);
        let temp = self.fetched >> 1;
        self.set_flag(Flag::Z, temp == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);

        match self.instructions[self.opcode as usize].addressing_mode {
            AM::IMP => self.a = temp,
            _ => self.write(self.addr_abs, temp),
        }
        0
    }

    fn op_nop(&mut self) -> u8 {
        match self.opcode {
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => 1,
            _ => 0,
        }
    }

    fn op_ora(&mut self) -> u8 {
        self.fetch();
        self.a |= self.fetched;
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        1
    }

    fn op_php(&mut self) -> u8 {
        self.write(
            0x0100 + self.sp as u16,
            self.status | Flag::B as u8 | Flag::U as u8,
        );
        self.sp -= 1;
        0
    }

    fn op_plp(&mut self) -> u8 {
        self.sp += 1;
        self.status = self.read(0x0100 + self.sp as u16);
        self.status &= !(Flag::U as u8);
        0
    }

    fn op_rol(&mut self) -> u8 {
        self.fetch();
        let temp = (self.fetched as u16) << 1 | self.get_flag(Flag::C) as u16;
        self.set_flag(Flag::C, (temp & 0xFF00) != 0);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);

        match self.instructions[self.opcode as usize].addressing_mode {
            AM::IMP => self.a = (temp & 0x00FF) as u8,
            _ => self.write(self.addr_abs, (temp & 0x00FF) as u8),
        }
        0
    }

    fn op_ror(&mut self) -> u8 {
        self.fetch();
        let temp = (self.get_flag(Flag::C) as u16) << 7 | (self.fetched as u16) >> 1;
        self.set_flag(Flag::C, (self.fetched & 0x01) != 0);
        self.set_flag(Flag::Z, (temp & 0x00FF) == 0);
        self.set_flag(Flag::N, (temp & 0x80) != 0);

        match self.instructions[self.opcode as usize].addressing_mode {
            AM::IMP => self.a = (temp & 0x00FF) as u8,
            _ => self.write(self.addr_abs, (temp & 0x00FF) as u8),
        }
        0
    }

    fn op_rts(&mut self) -> u8 {
        self.sp += 1;
        self.pc = self.read(0x0100 + self.sp as u16) as u16;
        self.sp += 1;
        self.pc |= (self.read(0x0100 + self.sp as u16) as u16) << 8;
        self.pc += 1;
        0
    }

    fn op_sec(&mut self) -> u8 {
        self._op_set_flag(Flag::C, true)
    }

    fn op_sed(&mut self) -> u8 {
        self._op_set_flag(Flag::D, true)
    }

    fn op_sei(&mut self) -> u8 {
        self._op_set_flag(Flag::I, true)
    }

    fn op_sta(&mut self) -> u8 {
        self.write(self.addr_abs, self.a);
        0
    }

    fn op_stx(&mut self) -> u8 {
        self.write(self.addr_abs, self.x);
        0
    }

    fn op_sty(&mut self) -> u8 {
        self.write(self.addr_abs, self.y);
        0
    }

    fn op_tax(&mut self) -> u8 {
        self.x = self.a;
        self.set_flag(Flag::Z, self.x == 0);
        self.set_flag(Flag::N, (self.x & 0x80) != 0);
        0
    }

    fn op_tay(&mut self) -> u8 {
        self.y = self.a;
        self.set_flag(Flag::Z, self.y == 0);
        self.set_flag(Flag::N, (self.y & 0x80) != 0);
        0
    }

    fn op_tsx(&mut self) -> u8 {
        self.x = self.sp;
        self.set_flag(Flag::Z, self.x == 0);
        self.set_flag(Flag::N, (self.x & 0x80) != 0);
        0
    }

    fn op_txa(&mut self) -> u8 {
        self.a = self.x;
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        0
    }

    fn op_txs(&mut self) -> u8 {
        self.sp = self.x;
        0
    }

    fn op_tya(&mut self) -> u8 {
        self.a = self.y;
        self.set_flag(Flag::Z, self.a == 0);
        self.set_flag(Flag::N, (self.a & 0x80) != 0);
        0
    }
}
