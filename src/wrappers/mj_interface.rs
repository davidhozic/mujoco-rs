//! MuJoCo user interface
//! This IS NOT YET TESTED.
use super::mj_rendering::MjrContext;
use crate::mujoco_c::*;

/******************************** */
//           Constants
/******************************** */
const MAX_UI_STORAGE_SIZE: usize = 256;  // Rust-specific constant for the location where the UI will write data.


/******************************** */
//           Enums
/******************************** */
/// Mouse button IDs used in the UI framework.
pub type MjtButton = mjtButton;

/// Event types used in the UI framework.
pub type MjtEvent = mjtEvent;

/// Item types used in the UI framework.
pub type MjtItem = mjtItem;

/// State of a UI section.
pub type MjtSection = mjtSection;


/***********************************************************************************************************************
** MjuiState
***********************************************************************************************************************/
pub type MjuiState = mjuiState;

impl Default for MjuiState{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

/***********************************************************************************************************************
** MjuiThemeSpacing
***********************************************************************************************************************/
pub type MjuiThemeSpacing = mjuiThemeSpacing;

impl Default for MjuiThemeSpacing{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}


/***********************************************************************************************************************
** MjuiThemeColor
***********************************************************************************************************************/
pub type MjuiThemeColor = mjuiThemeColor;

impl Default for MjuiThemeColor{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}


/***********************************************************************************************************************
** MjuiItem
***********************************************************************************************************************/
// pub enum MjuiItemModifier {
//     None = 0,
//     Control,
//     Shift,
//     Alt
// }

// impl MjuiItemModifier {
//     fn from_ffi(ffi: i32) -> Self {
//         match ffi {
//             0 => Self::None,
//             1 => Self::Control,
//             2 => Self::Shift,
//             4 => Self::Alt,
//             _ => unreachable!()
//         }
//     }
// }

#[derive(Clone)]
pub enum MjuiItemState {
    Disable = 0,
    Enable,
}

// /// Possible data types of the Edit widget.
// pub enum MjuiItemEditType {
//     String (String),
//     Integer (i32),
//     Double (f64),
// }

// /// Type of MuJoCo's UI widgets.
// pub enum MjuiItemType { 
//     Check { modifier: MjuiItemModifier, data: bool },
//     Radio { data: bool },
//     RadioLine { data: bool },
//     Select,
//     Slider { range: (f64, f64), divisions: f64, data: f64 },
//     Edit { range: (f64, f64), data: MjuiItemEditType },
//     Button,
//     Other
// }

// /// MuJoCo UI widget
// pub struct MjuiItem {
//     pub name: String,
//     pub state: MjuiItemState,
//     pub sectionid: u32,
//     pub itemid: u32,
//     pub userid: u32,
//     pub type_prop: MjuiItemType
// }

// impl MjuiItem {
//     pub(crate) fn from_ffi(ffi: &mjuiItem) -> Self {
//         /* Convert name */
//         let name_len = ffi.name.iter().position(|&x| x == 0).unwrap_or(ffi.name.len());
//         let u8_name = ffi.name.map(|x| x as u8);
//         let name = String::from_utf8_lossy(&u8_name[..name_len]).to_string();

//         /* Convert state */
//         let state = match ffi.state {
//             0 => MjuiItemState::Disable,
//             _ => MjuiItemState::Enable
//         };

//         use MjuiItemType::*;
//         use mjtItem::*;
        
//         let ffi_type_enum = ffi.type_.try_into().unwrap();
//         let type_prop = match ffi_type_enum {
//             mjITEM_CHECKBYTE | mjITEM_CHECKINT => {
//                 let single =  unsafe { ffi.__bindgen_anon_1.single.as_ref() };
//                 let data = if ffi.type_ == mjITEM_CHECKBYTE as i32 {
//                     unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 }
//                 }
//                 else {
//                     unsafe { *std::mem::transmute::<_, *mut mjtNum>(ffi.pdata).as_ref().unwrap() == 1.0 }
//                 };

//                 Check { modifier: MjuiItemModifier::from_ffi(single.modifier), data}
//             }

//             mjITEM_BUTTON => Button,

//             mjITEM_RADIO => Radio { data: unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 } },        

//             mjITEM_RADIOLINE => RadioLine { data: unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 } },    

//             mjITEM_SLIDERINT | mjITEM_SLIDERNUM => {
//                 let data = if ffi.type_ == mjITEM_SLIDERINT as i32 {
//                     unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() as f64 }
//                 }
//                 else {
//                     unsafe { *std::mem::transmute::<_, *mut f64>(ffi.pdata).as_ref().unwrap() }
//                 };

//                 let slider = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };

//                 Slider { range: slider.range.into(), divisions: slider.divisions, data }
//             }

//             mjITEM_EDITINT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
//                 let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
//                 let data = unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() };
//                 Edit { range: edit.range.into(), data: MjuiItemEditType::Integer(data)}
//             }

//             mjITEM_EDITNUM => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
//                 let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
//                 let data = unsafe { *std::mem::transmute::<_, *mut mjtNum>(ffi.pdata).as_ref().unwrap() };
//                 Edit { range: edit.range.into(), data: MjuiItemEditType::Double(data)}
//             }

//             mjITEM_EDITFLOAT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
//                 let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
//                 let data = unsafe { *std::mem::transmute::<_, *mut f32>(ffi.pdata).as_ref().unwrap() };
//                 Edit { range: edit.range.into(), data: MjuiItemEditType::Double(data as f64)}
//             }

//             mjITEM_EDITTXT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
//                 let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
//                 let data_char = unsafe { std::slice::from_raw_parts(ffi.pdata as *const u8, mjMAXUINAME as usize) };
//                 let data_len = data_char.iter().position(|&x| x == 0).unwrap_or(mjMAXUINAME as usize);
//                 let data = String::from_utf8_lossy(&data_char[..data_len]).to_string();

//                 Edit { range: edit.range.into(), data: MjuiItemEditType::String(data)}
//             }

//             _ => Other
//         };

//         Self {
//             name, state, sectionid: ffi.sectionid as u32, itemid: ffi.itemid as u32,
//             userid: ffi.userid as u32, type_prop
//         }
//     }
// }


/***********************************************************************************************************************
** MjuiSection
***********************************************************************************************************************/
pub type MjuiSection = mjuiSection;

impl MjuiSection {
    /// Returns the ``name`` attribute in Rust's ``String``.
    pub fn name(&self) -> String {
        let len = self.name.iter().position(|&x| x == 0).unwrap_or(mjMAXUINAME as usize);
        let u8_arr: [u8; mjMAXUINAME as usize] = self.name.map(|x| x as u8);
        String::from_utf8_lossy(&u8_arr[..len]).to_string()
    }

    /// Sets the ``name`` attribute from ``name``, which is Rust's ``String``.
    pub fn set_name(&mut self, name: String) {
        let by = name.as_bytes();
        assert!(by.len() < mjMAXUINAME as usize);
        unsafe { self.name[..by.len()].copy_from_slice(std::slice::from_raw_parts(by.as_ptr() as *const i8, by.len())); }
        self.name[by.len()] = 0;  // C string zero to mark the end of the string.
    }

    // pub fn items(&self) -> Box<[MjuiItem]> {
    //     let b: Box<[MjuiItem]> = self.item[..self.nitem as usize].iter().map(|&x| MjuiItem::from_ffi(x)).collect();
    //     b
    // }

    pub fn items(&mut self) -> &mut [mjuiItem] {
        &mut self.item[..self.nitem as usize]
    }
}


/***********************************************************************************************************************
** MjuiDef
***********************************************************************************************************************/
/// Widget specific configuration for the MjuiDef
pub enum MjuiDefType { 
    Check   { modifier: String },
    Radio   { values: String },
    Select  { values: String },
    Slider  { range: (f64, f64) },
    Edit    { range: (f64, f64), n_dimensions: usize, type_: MjuiDefDataType }, 
    Button  { modifier: String },
    End     // Ending element
}


pub enum MjuiDefDataType {
    String,
    Integer,
    Double,
}


pub struct MjuiDef {
    pub name: String,
    pub state: MjuiItemState,
    pub type_: MjuiDefType,

    // Internal data for writing values to, since Rust's borrow checker
    // doesn't allow MuJoCo's style of keeping a global pointer to some arbitrary data
    // while it gets modified by the UI.
    pub storage: [u8; MAX_UI_STORAGE_SIZE]
}

impl MjuiDef {
    /// Creates mjuiDef for usage in MuJoCo's UI functions.
    pub(crate) fn build_ffi_mut(&mut self) -> mjuiDef {
        let mut ffi = mjuiDef::default();
        let by = self.name.as_bytes();
        unsafe { ffi.name[..by.len()].copy_from_slice(std::slice::from_raw_parts(by.as_ptr() as *const i8, by.len())); }

        use MjuiDefType::*;
        let (type_, other) = match &self.type_ {
            Check { modifier } => {
                (mjtItem::mjITEM_CHECKBYTE, modifier)
            },

            Radio {values } => {
                (mjtItem::mjITEM_RADIO, values)
            },

            Select { values } => {
                (mjtItem::mjITEM_SELECT, values)
            },

            Edit { range, n_dimensions, type_ } => {
                let (t, r) = match type_ {
                    MjuiDefDataType::String => (MjtItem::mjITEM_EDITTXT, (0.0, 0.0)),
                    MjuiDefDataType::Integer => (MjtItem::mjITEM_EDITINT, *range),
                    MjuiDefDataType::Double => (MjtItem::mjITEM_EDITNUM, *range),
                };
                (t, &format!("{n_dimensions} {} {}", r.0, r.1))
            },

            Button { modifier } => {
                (mjtItem::mjITEM_BUTTON, modifier)
            },

            Slider { range } => {
                (mjtItem::mjITEM_SLIDERNUM, &format!("{} {}", range.0, range.1))
            },
            End => {
                (mjtItem::mjITEM_END, &"".to_string())
            }
        };

        let other_bytes = other.as_bytes();
        ffi.type_ = type_ as i32;
        ffi.state = self.state.clone() as i32;
        ffi.pdata = self.storage.as_mut_ptr() as *mut std::ffi::c_void;
        unsafe { ffi.other[..other_bytes.len()].copy_from_slice(std::slice::from_raw_parts(other_bytes.as_ptr() as *const i8, other_bytes.len())); }

        ffi
    }
}


impl Default for mjuiDef{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}


/***********************************************************************************************************************
** MjUI
***********************************************************************************************************************/
impl Default for mjUI{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

pub struct MjUI {
    ui: mjUI
}

impl MjUI {
    pub fn new() -> Self {
        Self{ ui: mjUI::default() }
    }

    pub fn add(&mut self, items: &mut [MjuiDef]) {
        let last_item = items.last();
        if let Some(x) = last_item {
            if let MjuiDefType::End = x.type_ {}
            else {
                panic!("last item in the items parameter must be MjuiDefType::End");
            }
        }

        let items: Box<[mjuiDef]> = items.iter_mut().map(|hdef| hdef.build_ffi_mut()).collect();
        unsafe { mjui_add(self.ffi_mut(), items.as_ptr()); }
    }

    pub fn add_to_section(&mut self, section_id: u32, items: &mut [MjuiDef]) {
        let items: Box<[mjuiDef]> = items.iter_mut().map(|hdef| hdef.build_ffi_mut()).collect();
        unsafe { mjui_addToSection(self.ffi_mut(), section_id as i32, items.as_ptr()); }
    }

    pub fn resize(&mut self, context: &MjrContext) {
        unsafe { mjui_resize(self.ffi_mut(), context.ffi()); }
    }

    /// Main update method. Must be called when the UI state changes or the user data changes.
    pub fn update(&self, section: Option<i32>, item: Option<i32>, state: &MjuiState, context: &MjrContext) {
        unsafe {
            mjui_update(
                section.unwrap_or(-1), item.unwrap_or(-1),
                self.ffi(),
                state, context.ffi()
            );
        }
    }

    // The event processing method. Creates a reference to mjuiItem.
    pub fn event(&mut self, state: &mut MjuiState, context: &MjrContext) -> &mut mjuiItem {
        let ffi_item = unsafe { mjui_event(self.ffi_mut(), state, context.ffi()) };
        unsafe {  ffi_item.as_mut().unwrap() }
    }

    pub fn render(&mut self, state: &MjuiState, context: &MjrContext) {
        unsafe { mjui_render(self.ffi_mut(), state, context.ffi()); }
    }

    pub fn ffi(&self) -> &mjUI {
        &self.ui
    }

    pub unsafe fn ffi_mut(&mut self) -> &mut mjUI {
        &mut self.ui
    }
}
