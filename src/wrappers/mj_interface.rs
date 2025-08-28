//! MuJoCo user interface
use crate::mujoco_c::*;

/******************************** */
//           Enums
/******************************** */
pub type MjtItem = mjtItem;


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
pub enum MjuiItemModifier {
    None = 0,
    Control,
    Shift,
    Alt
}

impl MjuiItemModifier {
    fn from_ffi(ffi: i32) -> Self {
        match ffi {
            0 => Self::None,
            1 => Self::Control,
            2 => Self::Shift,
            4 => Self::Alt,
            _ => unreachable!()
        }
    }
}

pub enum MjuiItemState {
    Disable = 0,
    Enable,
}

/// Possible data types of the Edit widget.
pub enum MjuiItemEditType {
    String (String),
    Integer (i32),
    Double (f64),
}

/// Type of MuJoCo's UI widgets.
pub enum MjuiItemType { 
    Check { modifier: MjuiItemModifier, data: bool },
    Radio { data: bool },
    RadioLine { data: bool },
    Select,
    Slider { range: (f64, f64), divisions: f64, data: f64 },
    Edit { range: (f64, f64), data: MjuiItemEditType },
    Button,
    Other
}

/// MuJoCo UI widget
pub struct MjuiItem {
    pub name: String,
    pub state: MjuiItemState,
    pub sectionid: u32,
    pub itemid: u32,
    pub userid: u32,
    pub type_prop: MjuiItemType
}

impl MjuiItem {
    pub(crate) fn from_ffi(ffi: mjuiItem) -> Self {
        /* Convert name */
        let name_len = ffi.name.iter().position(|&x| x == 0).unwrap_or(ffi.name.len());
        let u8_name = ffi.name.map(|x| x as u8);
        let name = String::from_utf8_lossy(&u8_name[..name_len]).to_string();

        /* Convert state */
        let state = match ffi.state {
            0 => MjuiItemState::Disable,
            _ => MjuiItemState::Enable
        };

        use MjuiItemType::*;
        use mjtItem::*;
        
        let ffi_type_enum = ffi.type_.try_into().unwrap();
        let type_prop = match ffi_type_enum {
            mjITEM_CHECKBYTE | mjITEM_CHECKINT => {
                let single =  unsafe { ffi.__bindgen_anon_1.single.as_ref() };
                let data = if ffi.type_ == mjITEM_CHECKBYTE as i32 {
                    unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 }
                }
                else {
                    unsafe { *std::mem::transmute::<_, *mut mjtNum>(ffi.pdata).as_ref().unwrap() == 1.0 }
                };

                Check { modifier: MjuiItemModifier::from_ffi(single.modifier), data}
            }

            mjITEM_BUTTON => Button,

            mjITEM_RADIO => Radio { data: unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 } },        

            mjITEM_RADIOLINE => RadioLine { data: unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() == 1 } },    

            mjITEM_SLIDERINT | mjITEM_SLIDERNUM => {
                let data = if ffi.type_ == mjITEM_SLIDERINT as i32 {
                    unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() as f64 }
                }
                else {
                    unsafe { *std::mem::transmute::<_, *mut f64>(ffi.pdata).as_ref().unwrap() }
                };

                let slider = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };

                Slider { range: slider.range.into(), divisions: slider.divisions, data }
            }

            mjITEM_EDITINT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
                let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
                let data = unsafe { *std::mem::transmute::<_, *mut i32>(ffi.pdata).as_ref().unwrap() };
                Edit { range: edit.range.into(), data: MjuiItemEditType::Integer(data)}
            }

            mjITEM_EDITNUM => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
                let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
                let data = unsafe { *std::mem::transmute::<_, *mut mjtNum>(ffi.pdata).as_ref().unwrap() };
                Edit { range: edit.range.into(), data: MjuiItemEditType::Double(data)}
            }

            mjITEM_EDITFLOAT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
                let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
                let data = unsafe { *std::mem::transmute::<_, *mut f32>(ffi.pdata).as_ref().unwrap() };
                Edit { range: edit.range.into(), data: MjuiItemEditType::Double(data as f64)}
            }

            mjITEM_EDITTXT => {  // | mjITEM_EDITNUM | mjITEM_EDITFLOAT | mjITEM_EDITTXT
                let edit = unsafe { ffi.__bindgen_anon_1.slider.as_ref() };
                let data_char = unsafe { std::slice::from_raw_parts(ffi.pdata as *const u8, mjMAXUINAME as usize) };
                let data_len = data_char.iter().position(|&x| x == 0).unwrap_or(mjMAXUINAME as usize);
                let data = String::from_utf8_lossy(&data_char[..data_len]).to_string();

                Edit { range: edit.range.into(), data: MjuiItemEditType::String(data)}
            }

            _ => Other
        };

        Self {
            name, state, sectionid: ffi.sectionid as u32, itemid: ffi.itemid as u32,
            userid: ffi.userid as u32, type_prop
        }
    }
}
