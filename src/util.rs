use crate::cpu::Flag;
use font8x8::legacy::BASIC_LEGACY;
use pretty_hex::*;

const LINE_H: u32 = 10;

pub fn draw_text_color(frame: &mut [u8], x: u32, y: u32, w: u32, text: &str, color: [u8; 4]) {
    let mut x = x;
    for c in text.chars() {
        draw_char_color(frame, x, y, w, c, color);
        x += 8;
    }
}

pub fn draw_text(frame: &mut [u8], x: u32, y: u32, w: u32, text: &str) {
    draw_text_color(frame, x, y, w, text, [255, 255, 255, 255]);
}

fn draw_char_color(frame: &mut [u8], x: u32, y: u32, w: u32, c: char, color: [u8; 4]) {
    let mut i = 0;
    for row in BASIC_LEGACY[c as usize] {
        let mut j = 0;
        for bit in 0..8 {
            if row & (1 << bit) != 0 {
                let i = ((x + j) + (y + i) * w) as usize * 4;
                frame[i..i + 4].copy_from_slice(&color);
            }
            j += 1;
        }
        i += 1;
    }
}

fn draw_flag_bit(frame: &mut [u8], x: u32, y: u32, w: u32, flag: char, status: bool) {
    let color = if status {
        [0x77, 0xDD, 0x77, 255]
    } else {
        [0xDD, 0x77, 0x77, 255]
    };

    draw_char_color(frame, x, y, w, flag, color);
}

fn draw_flags(frame: &mut [u8], x: u32, y: u32, w: u32, cpu: &crate::cpu::Cpu) {
    draw_text(frame, x, y, w, "S:");
    draw_flag_bit(frame, x + 24, y, w, 'N', cpu.get_flag(Flag::N));
    draw_flag_bit(frame, x + 32, y, w, 'V', cpu.get_flag(Flag::V));
    draw_char_color(frame, x + 40, y, w, '-', [0xDD, 0x77, 0x77, 255]);
    draw_flag_bit(frame, x + 48, y, w, 'B', cpu.get_flag(Flag::B));
    draw_flag_bit(frame, x + 56, y, w, 'D', cpu.get_flag(Flag::D));
    draw_flag_bit(frame, x + 64, y, w, 'I', cpu.get_flag(Flag::I));
    draw_flag_bit(frame, x + 72, y, w, 'Z', cpu.get_flag(Flag::Z));
    draw_flag_bit(frame, x + 80, y, w, 'C', cpu.get_flag(Flag::C));
}

pub fn draw_cpu_state(frame: &mut [u8], x: u32, y: u32, w: u32, cpu: &crate::cpu::Cpu) {
    draw_flags(frame, x, y, w, cpu);
    draw_text(frame, x, y + LINE_H * 1, w, &format!("A: {:02X}", cpu.a));
    draw_text(frame, x, y + LINE_H * 2, w, &format!("X: {:02X}", cpu.x));
    draw_text(frame, x, y + LINE_H * 3, w, &format!("Y: {:02X}", cpu.y));
    draw_text(frame, x, y + LINE_H * 4, w, &format!("PC: {:04X}", cpu.pc));
    draw_text(frame, x, y + LINE_H * 5, w, &format!("SP: {:04X}", cpu.sp));
}

pub fn draw_hex(frame: &mut [u8], x: u32, y: u32, w: u32, v: Vec<u8>, offset: usize, lines: usize) {
    let cfg: HexConfig = HexConfig {
        title: false,
        width: 4,
        group: 0,
        ascii: false,
        display_offset: offset,
        ..HexConfig::default()
    };

    let hex = v.hex_conf(cfg);

    let df = format!("{:?}", hex).replace("   ", " ").to_uppercase();

    for (i, line) in df.split("\n").enumerate().take(lines) {
        let mut parts = line.split(":");
        let addr = parts.next().unwrap().trim();
        let val = parts.next().unwrap().trim();
        draw_text_color(
            frame,
            x,
            y + LINE_H * i as u32,
            w,
            &addr,
            [150, 150, 150, 255],
        );
        draw_text(frame, x + 40, y + LINE_H * i as u32, w, val);
    }
}

pub fn draw_ram(frame: &mut [u8], x: u32, y: u32, w: u32, cpu: &crate::cpu::Cpu) {
    let v = cpu.bus.ram[0x0000..0x00FF].to_vec();
    draw_hex(frame, x, y, w, v, 0, 4);

    let v = cpu.bus.ram[0x8000..0x80FF].to_vec();
    draw_hex(frame, x, y + 50, w, v, 0x8000, 9);
}

pub fn draw_guide(frame: &mut [u8], x: u32, y: u32, w: u32) {
    draw_text(frame, x, y, w, "[S]tep [R]eset [Q]uit");
}
