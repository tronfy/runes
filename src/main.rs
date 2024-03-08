mod bus;
mod cpu;
mod instructions;
mod util;

use pixels::{Error, Pixels, SurfaceTexture};
use util::{draw_cpu_state, draw_guide, draw_ram, draw_text};
use winit::{
    dpi::LogicalSize,
    event::{Event, VirtualKeyCode},
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};
use winit_input_helper::WinitInputHelper;

const WIDTH: u32 = 256;
const HEIGHT: u32 = 240;

fn main() -> Result<(), Error> {
    let mut cpu = cpu::Cpu::new();

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
    cpu.cycles = 0;

    let event_loop = EventLoop::new();
    let mut input = WinitInputHelper::new();

    let window = {
        let size = LogicalSize::new(WIDTH as f64, HEIGHT as f64);
        let scaled_size = LogicalSize::new(WIDTH as f64 * 3.0, HEIGHT as f64 * 3.0);
        WindowBuilder::new()
            .with_title("runes")
            .with_inner_size(scaled_size)
            .with_min_inner_size(size)
            .build(&event_loop)
            .unwrap()
    };

    let mut pixels = {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(WIDTH, HEIGHT, surface_texture)?
    };

    event_loop.run(move |event, _, control_flow| {
        if let Event::RedrawRequested(_) = event {
            // clear to dark gray
            for p in pixels.frame_mut().chunks_exact_mut(4) {
                p.copy_from_slice(&[30, 30, 30, 255]);
            }

            draw_cpu_state(pixels.frame_mut(), 4, 4, WIDTH, &cpu);

            draw_ram(pixels.frame_mut(), 4, 74, WIDTH, &cpu);

            draw_guide(pixels.frame_mut(), 4, HEIGHT - 14, WIDTH);

            if let Err(err) = pixels.render() {
                println!("Error: {:?}", err);
                *control_flow = ControlFlow::Exit;
                return;
            }
        }

        if input.update(&event) {
            // exit
            if input.key_pressed(VirtualKeyCode::Q)
                || input.key_pressed(VirtualKeyCode::Escape)
                || input.close_requested()
            {
                *control_flow = ControlFlow::Exit;
                return;
            }

            // resize
            if let Some(size) = input.window_resized() {
                if let Err(err) = pixels.resize_surface(size.width, size.height) {
                    println!("pixels.resize_surface() failed: {err}");
                    *control_flow = ControlFlow::Exit;
                    return;
                }
            }

            // step
            if input.key_pressed(VirtualKeyCode::S) {
                loop {
                    cpu.clock();
                    if cpu.complete() {
                        break;
                    }
                }
            }

            // reset
            if input.key_pressed(VirtualKeyCode::R) {
                cpu.reset();
            }

            window.request_redraw();
        }

        window.request_redraw();
    });
}
