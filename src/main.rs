mod bus;
mod cpu;
mod instructions;

fn main() {
    let mut cpu = cpu::Cpu::new();

    // Load Program (assembled at https://www.masswerk.at/6502/assembler.html)
    /*
        *=$8000
        LDX #10
        STX $0000
        LDX #3
        STX $0001
        LDY $0000
        LDA #0
        CLC
        loop
        ADC $0001
        DEY
        BNE loop
        STA $0002
        NOP
        NOP
        NOP
    */
    let program = vec![
        0xA2, 0x0A, 0x8E, 0x00, 0x00, 0xA2, 0x03, 0x8E, 0x01, 0x00, 0xAC, 0x00, 0x00, 0xA9, 0x00,
        0x18, 0x6D, 0x01, 0x00, 0x88, 0xD0, 0xFA, 0x8D, 0x02, 0x00, 0xEA, 0xEA, 0xEA,
    ];
    let mut address: u16 = 0x8000;

    while address < 0x8000 + program.len() as u16 {
        cpu.bus.write(address, program[(address - 0x8000) as usize]);
        address += 1;
    }

    cpu.bus.write(0xFFFC, 0x00);
    cpu.bus.write(0xFFFD, 0x80);
    cpu.reset();

    println!();
    cpu.debug();
    println!();

    loop {
        loop {
            cpu.clock();
            if cpu.complete() {
                break;
            }
        }

        cpu.debug();
        println!();
    }
}
