//! Example demonstrating custom UI widgets in the MuJoCo viewer.
//! This example shows how to use the add_ui_callback method to create
//! custom windows, panels, and other UI elements using egui.

use std::time::Duration;
use std::cell::RefCell;
use std::rc::Rc;

use mujoco_rs::viewer::MjViewer;
use mujoco_rs::prelude::*;

const EXAMPLE_MODEL: &str = "
<mujoco>
  <worldbody>
    <light ambient=\"0.2 0.2 0.2\" pos=\".2 .2 .2\"/>
    <body name=\"ball\" pos=\"0 0 0.5\">
        <geom name=\"green_sphere\" size=\".1\" rgba=\"0 1 0 1\"/>
        <joint type=\"free\"/>
    </body>

    <geom name=\"floor\" type=\"plane\" size=\"10 10 1\" euler=\"5 0 0\"/>
  </worldbody>
</mujoco>
";


/// Example of a user-written simulation struct (minimal as possible).
struct CustomSimulation {
    data: MjData<Rc<MjModel>>,
    viewer: MjViewer<Rc<MjModel>>
}


/// Storage for keeping the custom UI state (per closure).
struct CallbackStorage {
    window_open: bool
}


impl CustomSimulation {
    fn new(model_xml_string: &str) -> Self {
        let model = Rc::new(MjModel::from_xml_string(model_xml_string).expect("could not load the model"));
        let data = MjData::new(model.clone());
        let mut viewer = MjViewer::launch_passive(model.clone(), 100)
            .expect("could not launch the viewer");

        // Common user-state storage, which can be used in multiple ui callbacks.
        // For callback local storage, simply use CallbackStorage instead of Rc<RefCell<CallbackStorage>>.
        let storage = Rc::new(RefCell::new(CallbackStorage { window_open: false }));

        // Example 1: Side panel with controls
        viewer.add_ui_callback({

            // Clone the reference counter so that the closure below moves the clone, not the original.
            let storage = storage.clone();
            move |ctx| {
                use mujoco_rs::viewer::egui;

                // Since we're using Rc, we need RefCell to dynamically borrow.
                let mut storage_borrow = storage.borrow_mut();

                // Bind key V to toggle the window.
                if ctx.input(|reader| reader.key_pressed(egui::Key::V)) {
                    storage_borrow.window_open = !storage_borrow.window_open;
                }

                // Process input events.
                ctx.input(|reader| {
                    for event in reader.events.iter() {
                        match event {
                            egui::Event::Key { key, pressed: true, ..} => {
                                println!("A new key has been pressed: {}", key.name());
                            }
                            _ => {}
                        }
                    }
                });

                // Create a side panel, with a button to toggle the window.
                egui::SidePanel::right("custom_panel")
                    .show(ctx, |ui| {
                        ui.heading("Custom Panel");
                        ui.separator();
                        ui.label("This is a custom side panel");
                        ui.label("Added via add_ui_callback!");
                        
                        if ui.button("Example Button").clicked() {
                            storage_borrow.window_open = !storage_borrow.window_open;
                        }
                    });

                // The window whose visibility can be toggled.
                egui::Window::new("Simulation Info")
                    .open(&mut storage_borrow.window_open)
                    .show(ctx, |ui| {
                        ui.heading("Simulation Information");
                        ui.separator();
                        ui.label("This is a custom information window.");
                        ui.label("You can add any egui widgets here!");
                    });
            }
        });

        // Example 2: Top panel
        viewer.add_ui_callback({
            // Clone the reference counter so that the closure below moves the clone, not the original.
            let storage = storage.clone();
            move |ctx| {
                use mujoco_rs::viewer::egui;

                // Since we're using Rc, we need RefCell to dynamically borrow.
                let storage_borrow = storage.borrow();
                egui::TopBottomPanel::top("custom_top_panel")
                    .show(ctx, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Custom Top Bar");
                            ui.separator();
                            ui.label("MuJoCo Rust Viewer with Custom Widgets");
                        });
                        ui.label(format!("Is window opened: {}", storage_borrow.window_open));
                    });
            }
        });


        Self { data, viewer }
    }

    fn viewer_running(&self) -> bool {
        self.viewer.running()
    }

    fn step(&mut self) {
        self.viewer.sync(&mut self.data);
        self.data.step();
    }
}


fn main() {
    let mut simulation = CustomSimulation::new(EXAMPLE_MODEL);
    
    // Run the simulation
    while simulation.viewer_running() {
        simulation.step();
        std::thread::sleep(Duration::from_millis(2));
    }
}
