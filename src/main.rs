#![allow(non_snake_case)]
use std::f32::consts;
use std::ops::*;
use std::io::{self, Stdout, Write, Read};
use std::thread;
use std::fs::File;
use std::time::{Duration, Instant};
use std::cmp::min;
use termion::raw::IntoRawMode;


fn main() {
    let main = Wing::new(100.0, 7.0, 0.0);
    let tail = Wing::new(5.0, 2.0, -0.05);
    let mut plane = Aircraft::new(main, tail, 0.3, 0.9, 0.15, 1500.0, 3000.0);

    let mut stdout = io::stdout().into_raw_mode().unwrap();
    let mut stdin = termion::async_stdin();
    
    let mut file = File::options().append(true).open("log.log").unwrap();
    writeln!(file, "x vel,z vel,z pos,alpha,alpha_0t,gamma,gamma_v,drag,thrust,lift,moment,rho").unwrap();

    loop {
        plane.dynamics();
        //print_state(&plane, &mut stdout);
        pretty_print(&plane, &mut stdout);
        plane.calculate_moments();
        let mut buf = [0];
        let _ = stdin.read(&mut buf);
        handle_key_down(&buf, &mut plane);
        thread::sleep(Duration::from_millis(10));
    }
}

fn print_state(aircraft: &Aircraft, stdout: &mut Stdout) {
    write!(stdout, "{}", termion::clear::All).unwrap();
    write!(stdout, "{}throttle input: {:.2}", termion::cursor::Goto(2, 1), aircraft.controls.throttle).unwrap();


    write!(stdout, "{}velocity components: {:.1}, {:.1}, {:.1}", termion::cursor::Goto(2, 19), aircraft.state.vel.x, aircraft.state.vel.y, aircraft.state.vel.z).unwrap();
    write!(stdout, "{}V_mag: {:.2}", termion::cursor::Goto(2, 20), aircraft.state.vel.magnitude()).unwrap();
    write!(stdout, "{}position: {:.1}, {:.1}, {:.1}", termion::cursor::Goto(2, 20), aircraft.state.position.x, aircraft.state.position.y, aircraft.state.position.z).unwrap();

    write!(stdout, "{}main lift: {:.1} tail lift: {:.1}", termion::cursor::Goto(2, 21), aircraft.main.lift, aircraft.tail.lift).unwrap();

    write!(stdout, "{}alpha {:.1}", termion::cursor::Goto(2, 22), aircraft.state.alpha * 180.0/consts::PI).unwrap();
    write!(stdout, "{}elevator angle {:.1}", termion::cursor::Goto(2, 23), aircraft.tail.alpha_0).unwrap();
    write!(stdout, "{}moment {:.1}", termion::cursor::Goto(2, 24), aircraft.calculate_moments()).unwrap();


    stdout.flush().unwrap();
}

fn pretty_print(ac: &Aircraft, out: &mut Stdout) {
    let ibox = "█";
    write!(out, "{}{}", termion::clear::All, termion::cursor::Hide).unwrap();
    let (x_s, y_s) = termion::terminal_size().unwrap();

    write!(out, "{}throttle", termion::cursor::Goto(x_s - 10, 1)).unwrap();
    write!(out, "{}{}{:.0}{}%", termion::cursor::Goto(x_s - 10, 2), termion::color::Fg(termion::color::Yellow), ac.controls.throttle * 100.0, termion::color::Fg(termion::color::White)).unwrap();
    box_ind(out, x_s - 10, 4, 0.1, 1.0, false, 2, ac.controls.throttle);

    write!(out, "{}alpha", termion::cursor::Goto(x_s - 20, 1)).unwrap();
    write!(out, "{}{}{:.2}{}%", termion::cursor::Goto(x_s - 20, 2), termion::color::Fg(termion::color::Yellow), ac.state.alpha * 180.0/consts::PI, termion::color::Fg(termion::color::White)).unwrap();
    box_ind(out, x_s - 20, 4, 0.02, 0.2, false, 2, ac.state.alpha);

    write!(out, "{}{:.1}", termion::cursor::Goto(10, y_s - 12), ac.state.position.z).unwrap();
    write!(out, "{}alt (m)", termion::cursor::Goto(12, y_s - 8)).unwrap();
    dial(out, 15, y_s - 6, 5, 500.0, false, ac.state.position.z);

    write!(out, "{}{}{:.1}{}", termion::cursor::Goto(30, y_s - 12), termion::color::Fg(termion::color::Yellow), ac.state.vel.magnitude(), termion::color::Fg(termion::color::White)).unwrap();
    write!(out, "{}spd (m/s)", termion::cursor::Goto(36, y_s - 8)).unwrap();
    dial(out, 40, y_s - 6, 5, 100.0, true, ac.state.vel.magnitude());

    write!(out, "{}{}{:.1}{}", termion::cursor::Goto(65, y_s - 12), termion::color::Fg(termion::color::Yellow), ac.state.vel.z, termion::color::Fg(termion::color::White)).unwrap();
    write!(out, "{}v_spd (m/s)", termion::cursor::Goto(61, y_s - 8)).unwrap();
    dial(out, 65, y_s - 6, 3, 50.0, true, ac.state.vel.z);






    out.flush().unwrap();
}

fn box_ind(out: &mut Stdout, x: u16, y: u16, interval: f32, high: f32, horizontal: bool, spacing: u16, value: f32) {
    let ibox = "█";
    let hmark = "|";
    let vmark = "_";

    let num_marks = (high/interval) as u16;

    for i in 0..num_marks {
        if horizontal {
            write!(out, "{}{}", termion::cursor::Goto(i*spacing + x, y), hmark).unwrap();
        }else{
            write!(out, "{}{}", termion::cursor::Goto(x, i*spacing + y), vmark).unwrap();
        }
    }

    let num_lights = (spacing as f32 *value/interval) as u16;

    write!(out, "{}", termion::color::Fg(termion::color::Yellow)).unwrap();
    for i in 0..num_lights{
        if horizontal {
            write!(out, "{}{}", termion::cursor::Goto(i+x, y+1), ibox).unwrap();
        }else{
            write!(out, "{}{}", termion::cursor::Goto(x+1, y + num_marks * spacing -i), ibox).unwrap();
        }
    }
    write!(out, "{}", termion::color::Fg(termion::color::White)).unwrap();

}

fn dial(out: &mut Stdout, x: u16, y: u16, r: u16, high: f32, horizontal: bool, value: f32) {
    let circ = "●";

    let angle = 2.0*consts::PI*value/high;

    let dx: f32;
    let dy: f32;
    if horizontal {
        dx = -angle.cos();
        dy = -angle.sin();
    } else{
        dy = -angle.cos();
        dx = angle.sin();
    }
    
    for a in 0..r+1 {
        let xh = 2 * ((r.pow(2) - a.pow(2)) as f32).sqrt() as u16; // i have no idea why that 2
                                                                   // needs to be there 
        write!(out, "{}{}", termion::cursor::Goto(x+xh, y+a), circ).unwrap();
        write!(out, "{}{}", termion::cursor::Goto(x-xh, y+a), circ).unwrap();
        write!(out, "{}{}", termion::cursor::Goto(x-xh, y-a), circ).unwrap();
        write!(out, "{}{}", termion::cursor::Goto(x+xh, y-a), circ).unwrap();
    }

    write!(out, "{}", termion::color::Fg(termion::color::Yellow)).unwrap();
    for a in 0..r+1 {
        let i = a as f32;
        write!(out,"{}{}", termion::cursor::Goto((dx * i + x as f32) as u16, (dy * i + y as f32) as u16), circ).unwrap();
    }
    write!(out, "{}", termion::color::Fg(termion::color::White)).unwrap();
}

fn handle_key_down(stdin: &[u8], aircraft: &mut Aircraft) {
    for c in stdin{
        match c {
            // these are ASCII codes
            0 => aircraft.controls.normal(),
            114 => aircraft.controls.increase_throttle(),
            102 => aircraft.controls.decrease_throttle(),
            113 => aircraft.controls.roll_left(),
            101 => aircraft.controls.roll_right(),
            115 => aircraft.controls.pitch_up(),
            119 => aircraft.controls.pitch_down(),
            255 => continue,
            _ => todo!("{c}")
          
        }
    }
}

struct Wing {
    /// struct to handle all of the aerodynamics of a wing 
    ///
    /// S is surface area in m^2
    /// A is aspect ratio in m^2 (expect 2-7)
    /// alpha_0 is setting angle/simulates camber in radians
    /// clalpha is the lift curve slope in rad^-1
    
    S: f32,
    A: f32,
    alpha_0: f32,
    CLalpha: f32,
    lift: f32,
    drag: f32,
}

impl Wing {
    fn new(S: f32, A: f32, alpha_0: f32) -> Wing {
        let mut thingy = Wing {
            S,
            A,
            alpha_0,
            CLalpha: 0.0,
            lift: 0.0,
            drag: 0.0,
        };
        thingy.find_cla();
        return thingy
    }

    fn find_cla(&mut self) {
        /// finds the lift curve slope from prandtl's lifting line theory, assumes a high aspect
        /// ratio wing

        let val = (2.0*consts::PI)/(1.0 + (2.0*consts::PI/(consts::PI * self.A)));
        self.CLalpha = val;
    }

    fn aero_forces(&mut self, state: &State) {
        let mut alpha_eff = self.alpha_0 + state.alpha;

        // stall
        if alpha_eff > 1.2 {
            alpha_eff = 0.8;
        }
        let CL = self.CLalpha * alpha_eff;

        self.lift = 0.5 * state.density_from_alt() * state.vel.magnitude().powf(2.0) * self.S * CL;

        let CD = CL.powf(2.0)/(consts::PI * self.A);

        self.drag = (0.5 * state.density_from_alt() * state.vel.magnitude().powf(2.0) * self.S * CD).abs();

    }
}

struct Aircraft {
    /// struct to hold all of the parts of the plane
    ///
    /// main and tail are structs to represent wings
    /// length is the length of the plane in meters
    /// le_frac is a fraction to show how far the leading edge of the main wing is from the nose
    /// le_frac_t is the fraction of the length from the nose to the leading edge of the tail
    /// mass is the mass in kg
    /// state stores what the plane is doing
    /// x_cg is the percentage of the aircraft that the centre of gravity is
    
    main: Wing,
    tail: Wing,
    le_frac: f32,
    le_frac_t: f32,
    mass: f32,
    state: State,
    controls: Controls,
    engine: Propulsion,
    x_cg: f32,
}

impl Aircraft {
    fn new(main: Wing, tail: Wing, le_frac: f32, le_frac_t: f32, x_cg: f32, mass: f32, power: f32) -> Aircraft {
        if le_frac > 1.0 {
            println!("main wing is very far back!");
        } else if le_frac < 0.0 {
            println!("main wing is very far forwards!")
        }
        
        if le_frac_t > 1.0 {
            println!("tail is very far back!");
        } else if le_frac_t < 0.0 {
            println!("tail is very far forwards!")
        }

        let vel = Vector::new(40.0, 0.0, 0.0);
        let pos = Vector::new(0.0, 0.0, 0.0);

        let situation = State{vel: vel, position: pos, alpha: 0.0, phi: 0.0, heading: 0.0, time: Instant::now(), gamma: 0.0, gammadot: 0.0};


        return Aircraft{
            main,
            tail,
            le_frac,
            le_frac_t,
            x_cg,
            mass,
            state: situation,
            controls: Controls::new(),
            engine: Propulsion{power},
        }
    }

    fn dynamics(&mut self) {
        let dt = self.state.time.elapsed().as_secs_f32();
        self.state.time = Instant::now();
    
        self.main.aero_forces(&self.state);
        self.tail.aero_forces(&self.state);

                                       // squared. the constant is the rotational inertia I 

        let t = self.engine.power * self.controls.throttle * self.state.density_from_alt(); 

        let forces_body = Vector::new(t - (self.tail.drag + self.main.drag), 0.0, self.main.lift + self.tail.lift);

        let forces_world = Vector::new( // transform from body coordinate system to world
                                        // coordinate system
            (forces_body.x * self.state.gamma.cos()) - (forces_body.z * self.state.gamma.sin()), 
            0.0,
            (forces_body.z * self.state.gamma.cos()) + (forces_body.x * self.state.gamma.sin()) - (self.mass * 9.81)
        );

        let accels_w = forces_world / self.mass;
        self.state.vel = &self.state.vel + &(&accels_w * dt);
        self.state.position = &self.state.position + &(&self.state.vel * dt);



        let m_tot = self.calculate_moments();

        /*if self.state.position.z < 0.0 {
            if self.state.vel.z < 0.0 {
                self.state.vel.z = 0.0;
                self.state.position.z = 0.0;
            }
            if m_tot < 10000.0 {
                m_tot = 0.0;
            }
            self.state.gamma = 0.0;
        }
        */

        self.state.gammadot -= m_tot * dt / 250.0; // the moment is an angular acceleration so it has to be dt
        self.state.gamma += self.state.gammadot * dt; // the moment is an angular acceleration so it has to be dt
        let mut gamma_v = (self.state.vel.z/self.state.vel.x).atan();
        if gamma_v.is_nan() {
            gamma_v = 0.1;
        }
        self.state.alpha = self.state.gamma - gamma_v;

        while self.state.gamma > consts::PI {
            self.state.gamma -= 2.0*consts::PI;
        }
        while self.state.gamma < -consts::PI {
            self.state.gamma += 2.0*consts::PI;
        }

        self.tail.alpha_0 -= 2.0 * self.controls.elevator * dt;

        let mut file = File::options().append(true).open("log.log").unwrap();
        writeln!(file, "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}", self.state.vel.x, self.state.vel.z, self.state.position.z, self.state.alpha, self.tail.alpha_0, self.state.gamma, gamma_v, self.main.drag + self.tail.drag, t, (self.main.lift + self.tail.lift)/(self.mass*9.81), m_tot, self.state.density_from_alt()).unwrap();

        if self.state.alpha.abs() > consts::PI * 2.0 {
            panic!();
        }


    }

    fn calculate_moments(&self) -> f32 {
        // find moments about the leading edge
        let cw = self.main.lift * (self.x_cg - self.le_frac);
        let ccw = self.tail.lift * (self.le_frac_t - self.x_cg);
        ccw - cw
    }
}

#[derive(Debug)]
#[allow(dead_code)]
struct State {
    /// the state of the aircraft in the world
    ///
    /// Has a velocity in body coordinates (m s^-1)
    /// a position in world coordinates (m)
    /// angle of attack (radians)
    /// bank angle/roll (radianscoefficent of lift,)
    /// heading in world units (degrees)
    
    vel: Vector,
    position: Vector,
    alpha: f32,
    gamma: f32,
    phi: f32,
    heading: f32,
    time: Instant, 
    gammadot: f32,

}

impl State{
    fn density_from_alt(&self) -> f32 {
        (self.position.z / 1000.0).cos() + 5 * self.position.x.sin()
    }
}

#[derive(Debug)]
struct Vector {
    /// 3D vector

    x: f32,
    y: f32,
    z: f32,
}

#[allow(dead_code)]
impl Vector {
    fn new (x: f32, y: f32, z: f32) -> Vector {
        return Vector{x, y, z}
    }

    fn copy(&self) -> Vector {
        Vector{
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    fn magnitude(&self) -> f32 {
        (self.x.powf(2.0) + self.y.powf(2.0) + self.z.powf(2.0)).powf(0.5)
    }

    fn normalize(&self) -> Vector {
        let m = self.magnitude().max(0.001);
        Vector{
            x: self.x/m,
            y: self.y/m,
            z: self.z/m,
        }
    }

    fn dot(&self, other: &Vector) -> Vector {
        Vector{
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,

        }
    }

}

/*impl<'a, 'b> Add<&'b Vector> for &'a Vector {
    type Output = Vector;

    fn add(self, other: &'b Vector) -> Vector {
        Vector {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}*/

impl Add for &Vector {
    type Output = Vector;

    fn add(self, other: &Vector) -> Vector {
        Vector {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vector {
    type Output = Vector;

    fn sub(self, other: Vector) -> Vector {
        Vector {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for &Vector {
    type Output = Vector;

    fn mul(self, other: f32) -> Vector {
        Vector{
            x: self.x * other,
            y: self.y * other,
            z: self.z * other,
        }
    }
}

impl Div<f32> for Vector {
    type Output = Vector;

    fn div(self, other: f32) -> Vector {
        Vector{
            x: self.x / other,
            y: self.y / other,
            z: self.z / other,
        }
    }
}

struct Propulsion {
    /// type of engine
    ///
    /// power is engine power in watts

    power: f32,
}

struct Controls {
    /// struct of control inputs, all ranging from -1 to 1
    
    elevator: f32,
    rudder: f32,
    aileron: f32,
    throttle: f32,
}

impl Controls {
    fn new() -> Controls {
        Controls {
            elevator: 0.0,
            rudder: 0.0,
            aileron: 0.0,
            throttle: 1.0,
        }
    }

    fn normal(&mut self) {
        self.elevator = 0.0;
        self.aileron = 0.0;
        self.rudder = 0.0;
    }

    fn increase_throttle(&mut self) {
        self.throttle += 0.05;
        self.throttle = self.throttle.min(1.0);
    }

    fn decrease_throttle(&mut self) {
        self.throttle -= 0.05;
        self.throttle = self.throttle.max(0.0);
    }

    fn roll_right(&mut self) {
        self.aileron = 1.0;
    }

    fn roll_left(&mut self) {
        self.aileron = -1.0;
    }

    fn pitch_up(&mut self) {
        self.elevator = 1.0;
    }


    fn pitch_down(&mut self) {
        self.elevator = -1.0;
    }


}
