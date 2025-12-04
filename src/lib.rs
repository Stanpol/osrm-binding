pub mod algorithm;
pub mod errors;
pub mod tables;
pub mod trip;
pub mod point;
pub mod route;
pub mod waypoints;
pub mod osrm_engine;
pub mod r#match;
pub mod nearest;
// src/lib.rs
use std::ffi::{c_void, CStr, CString};
use std::os::raw::c_char;

#[repr(C)]
struct OsrmResult {
    code: i32,
    message: *mut c_char,
}

#[repr(C)]
pub struct OsrmConfig {
    algorithm: *const c_char,
    shared_memory: bool,
    dataset_name: *const c_char,
    mmap_memory: bool,
    path: *const c_char,
    disable_feature_dataset_flags: i32,
    max_locations_trip: i32,
    max_locations_viaroute: i32,
    max_locations_distance_table: i32,
    max_locations_map_matching: i32,
    max_radius_map_matching: f64,
    max_results_nearest: i32,
    max_alternatives: i32,
    default_radius: f64,
}

#[link(name = "osrm_wrapper", kind = "static")]
unsafe extern "C" {
    fn osrm_create(base_path: *const c_char, algorithm : *const c_char, max_table_size: i32) -> *mut c_void;
    fn osrm_create_with_config(config: *const OsrmConfig) -> *mut c_void;
    fn osrm_destroy(osrm_instance: *mut c_void);
    fn osrm_table(
        osrm_instance: *mut c_void,
        coordinates: *const f64,
        num_coordinates: usize,
        sources: *const usize,
        num_sources: usize,
        destinations: *const usize,
        num_destinations: usize,
        include_duration: bool,
        include_distance: bool,
        bearings: *const f64,
        num_bearings: usize,
        radiuses: *const f64,
        num_radiuses: usize,
        hints: *const *const c_char,
        num_hints: usize,
        generate_hints: bool,
        approaches: *const *const c_char,
        num_approaches: usize,
        fallback_speed: f64,
        fallback_coordinate: *const c_char,
        scale_factor: f64,
        snapping: *const c_char,
    ) -> OsrmResult;

    fn osrm_trip(
        osrm_instance: *mut c_void,
        coordinates: *const f64,
        num_coordinates: usize
    ) -> OsrmResult;

    fn osrm_route(
        osrm_instance: *mut c_void,
        coordinates: *const f64,
        num_coordinates: usize,
        bearings: *const f64,
        num_bearings: usize,
        radiuses: *const f64,
        num_radiuses: usize,
        hints: *const *const c_char,
        num_hints: usize,
        generate_hints: bool,
        approaches: *const *const c_char,
        num_approaches: usize,
        snapping: *const c_char,
    ) -> OsrmResult;
    
    fn osrm_match(
        osrm_instance: *mut c_void,
        coordinates: *const f64,
        num_coordinates: usize,
        timestamps: *const u32,
        num_timestamps: usize,
        radiuses: *const f64,
        num_radiuses: usize,
        bearings: *const f64,
        num_bearings: usize,
        hints: *const *const c_char,
        num_hints: usize,
        generate_hints: bool,
        approaches: *const *const c_char,
        num_approaches: usize,
        gaps: *const c_char,
        tidy: bool,
        waypoints: *const usize,
        num_waypoints: usize,
        snapping: *const c_char,
    ) -> OsrmResult;

    fn osrm_nearest(
        osrm_instance: *mut c_void,
        coordinates: *const f64,
        num_coordinates: usize,
        bearings: *const f64,
        num_bearings: usize,
        radiuses: *const f64,
        num_radiuses: usize,
        hints: *const *const c_char,
        num_hints: usize,
        generate_hints: bool,
        number: i32,
        approaches: *const *const c_char,
        num_approaches: usize,
        snapping: *const c_char,
    ) -> OsrmResult;
    
    fn osrm_free_string(s: *mut c_char);
}

/// Configuration options for creating an OSRM engine instance
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// The algorithm to use for routing. Can be 'CH' or 'MLD'
    pub algorithm: Option<String>,
    /// Connects to the persistent shared memory datastore
    pub shared_memory: bool,
    /// Dataset name for shared memory datastore
    pub dataset_name: Option<String>,
    /// Map on-disk files to virtual memory addresses (mmap)
    pub mmap_memory: bool,
    /// The path to the .osrm files
    pub path: Option<String>,
    /// Disables feature datasets: ROUTE_STEPS, ROUTE_GEOMETRY
    pub disable_feature_dataset: Vec<String>,
    /// Max locations supported in trip query
    pub max_locations_trip: Option<i32>,
    /// Max locations supported in viaroute query
    pub max_locations_viaroute: Option<i32>,
    /// Max locations supported in distance table query
    pub max_locations_distance_table: Option<i32>,
    /// Max locations supported in map-matching query
    pub max_locations_map_matching: Option<i32>,
    /// Max radius size supported in map matching query
    pub max_radius_map_matching: Option<f64>,
    /// Max results supported in nearest query
    pub max_results_nearest: Option<i32>,
    /// Max number of alternatives supported in alternative routes query
    pub max_alternatives: Option<i32>,
    /// Default radius for queries
    pub default_radius: Option<f64>,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            algorithm: Some("CH".to_string()),
            shared_memory: true,
            dataset_name: None,
            mmap_memory: false,
            path: None,
            disable_feature_dataset: Vec::new(),
            max_locations_trip: None,
            max_locations_viaroute: None,
            max_locations_distance_table: None,
            max_locations_map_matching: None,
            max_radius_map_matching: Some(5.0),
            max_results_nearest: None,
            max_alternatives: Some(3),
            default_radius: None,
        }
    }
}

pub(crate) struct Osrm {
    instance: *mut c_void,
    _algorithm: Option<CString>,
    _dataset_name: Option<CString>,
    _path: Option<CString>,
}

impl Osrm {
    #[allow(dead_code)]
    pub(crate) fn new(base_path: &str, algorithm: &str, max_table_size: i32) -> Result<Self, String> {
        Self::new_with_options(base_path, algorithm, max_table_size)
    }

    pub(crate) fn new_with_options(base_path: &str, algorithm: &str, max_table_size: i32) -> Result<Self, String> {
        let c_path = CString::new(base_path).map_err(|e| e.to_string())?;
        let c_algorithm = CString::new(algorithm).map_err(|e| e.to_string())?;
        let instance = unsafe { osrm_create(c_path.as_ptr(), c_algorithm.as_ptr(), max_table_size) };

        if instance.is_null() {
            Err("Failure to create an OSRM instance.".to_string())
        } else {
            Ok(Osrm { 
                instance,
                _algorithm: Some(c_algorithm),
                _dataset_name: None,
                _path: Some(c_path),
            })
        }
    }

    pub(crate) fn new_with_config(config: EngineConfig) -> Result<Self, String> {
        // Convert strings to CStrings and keep them alive
        let c_algorithm = config.algorithm.as_ref()
            .map(|s| CString::new(s.as_str()).map_err(|e| e.to_string()))
            .transpose()?;
        
        let c_dataset_name = config.dataset_name.as_ref()
            .map(|s| CString::new(s.as_str()).map_err(|e| e.to_string()))
            .transpose()?;
        
        let c_path = config.path.as_ref()
            .map(|s| CString::new(s.as_str()).map_err(|e| e.to_string()))
            .transpose()?;

        // Calculate disable_feature_dataset_flags bitfield
        let mut disable_flags = 0i32;
        for feature in &config.disable_feature_dataset {
            match feature.as_str() {
                "ROUTE_STEPS" => disable_flags |= 1,
                "ROUTE_GEOMETRY" => disable_flags |= 2,
                _ => {}
            }
        }

        let ffi_config = OsrmConfig {
            algorithm: c_algorithm.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            shared_memory: config.shared_memory,
            dataset_name: c_dataset_name.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            mmap_memory: config.mmap_memory,
            path: c_path.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            disable_feature_dataset_flags: disable_flags,
            max_locations_trip: config.max_locations_trip.unwrap_or(0),
            max_locations_viaroute: config.max_locations_viaroute.unwrap_or(0),
            max_locations_distance_table: config.max_locations_distance_table.unwrap_or(0),
            max_locations_map_matching: config.max_locations_map_matching.unwrap_or(0),
            max_radius_map_matching: config.max_radius_map_matching.unwrap_or(0.0),
            max_results_nearest: config.max_results_nearest.unwrap_or(0),
            max_alternatives: config.max_alternatives.unwrap_or(0),
            default_radius: config.default_radius.unwrap_or(0.0),
        };

        let instance = unsafe { osrm_create_with_config(&ffi_config) };

        if instance.is_null() {
            Err("Failure to create an OSRM instance.".to_string())
        } else {
            Ok(Osrm {
                instance,
                _algorithm: c_algorithm,
                _dataset_name: c_dataset_name,
                _path: c_path,
            })
        }
    }

    pub(crate) fn trip(&self, coordinates: &[(f64, f64)]) -> Result<String, String> {

        let coords : Vec<f64> = coordinates.iter().flat_map(|&(lon, lat)| vec![lon, lat]).collect();
        let result = unsafe {
            osrm_trip(self.instance, coords.as_ptr(), coordinates.len() )
        };

        let message_ptr = result.message;
        if message_ptr.is_null() {
            return Err("OSRM returned a null message".to_string());
        }

        let c_str = unsafe { CStr::from_ptr(message_ptr) };
        let rust_str = c_str.to_str().map_err(|e| e.to_string())?.to_owned();

        unsafe {
            osrm_free_string(message_ptr);
        }

        if result.code != 0 {
            return Err(format!("OSRM error: {}", rust_str));
        }

        Ok(rust_str)
    }

    pub(crate) fn route(
        &self,
        coordinates: &[(f64, f64)],
        bearings: Option<&[(f64, f64)]>,
        radiuses: Option<&[f64]>,
        hints: Option<&[Option<String>]>,
        generate_hints: bool,
        approaches: Option<&[Option<String>]>,
        snapping: Option<&str>,
    ) -> Result<String, String> {
        let coords: Vec<f64> = coordinates.iter().flat_map(|&(lon, lat)| vec![lon, lat]).collect();
        
        // Prepare bearings
        let bearings_vec = bearings.map(|b| {
            b.iter().flat_map(|&(v, r)| vec![v, r]).collect::<Vec<f64>>()
        });
        let bearings_ptr = bearings_vec.as_ref().map(|v| v.as_ptr()).unwrap_or(std::ptr::null());
        let num_bearings = bearings.map(|b| b.len()).unwrap_or(0);
        
        // Prepare radiuses
        let radiuses_ptr = radiuses.map(|r| r.as_ptr()).unwrap_or(std::ptr::null());
        let num_radiuses = radiuses.map(|r| r.len()).unwrap_or(0);
        
        // Prepare hints (convert Option<String> to C strings)
        let hints_cstrings: Option<Vec<CString>> = hints.map(|h| {
            h.iter().map(|opt_s| {
                match opt_s {
                    Some(s) => CString::new(s.as_str()).unwrap_or_else(|_| CString::new("").unwrap()),
                    None => CString::new("").unwrap(),
                }
            }).collect()
        });
        let hints_ptrs: Option<Vec<*const c_char>> = hints_cstrings.as_ref().map(|cs| {
            cs.iter().map(|c| c.as_ptr()).collect()
        });
        let hints_ptr = hints_ptrs.as_ref().map(|p| p.as_ptr()).unwrap_or(std::ptr::null());
        let num_hints = hints.map(|h| h.len()).unwrap_or(0);
        
        // Prepare approaches (convert Option<String> to C strings)
        let approaches_cstrings: Option<Vec<CString>> = approaches.map(|a| {
            a.iter().map(|opt_s| {
                match opt_s {
                    Some(s) => CString::new(s.as_str()).unwrap_or_else(|_| CString::new("").unwrap()),
                    None => CString::new("").unwrap(),
                }
            }).collect()
        });
        let approaches_ptrs: Option<Vec<*const c_char>> = approaches_cstrings.as_ref().map(|cs| {
            cs.iter().map(|c| c.as_ptr()).collect()
        });
        let approaches_ptr = approaches_ptrs.as_ref().map(|p| p.as_ptr()).unwrap_or(std::ptr::null());
        let num_approaches = approaches.map(|a| a.len()).unwrap_or(0);
        
        // Prepare snapping
        let snapping_cstring = snapping.map(|s| CString::new(s).unwrap());
        let snapping_ptr = snapping_cstring.as_ref().map(|c| c.as_ptr()).unwrap_or(std::ptr::null());

        let result = unsafe {
            osrm_route(
                self.instance,
                coords.as_ptr(),
                coordinates.len(),
                bearings_ptr,
                num_bearings,
                radiuses_ptr,
                num_radiuses,
                hints_ptr,
                num_hints,
                generate_hints,
                approaches_ptr,
                num_approaches,
                snapping_ptr,
            )
        };

        let message_ptr = result.message;
        if message_ptr.is_null() {
            return Err("OSRM returned a null message".to_string());
        }

        let c_str = unsafe { CStr::from_ptr(message_ptr) };
        let rust_str = c_str.to_str().map_err(|e| e.to_string())?.to_owned();

        unsafe {
            osrm_free_string(message_ptr);
        }

        if result.code != 0 {
            return Err(format!("OSRM error: {}", rust_str));
        }

        Ok(rust_str)
    }

    pub(crate) fn table(
        &self,
        coordinates: &[(f64, f64)],
        sources: Option<&[usize]>,
        destinations: Option<&[usize]>,
        include_duration: bool,
        include_distance: bool,
        bearings: Option<&[(f64, f64)]>,
        radiuses: Option<&[f64]>,
        hints: Option<&[Option<String>]>,
        generate_hints: bool,
        approaches: Option<&[Option<String>]>,
        fallback_speed: Option<f64>,
        fallback_coordinate: Option<&str>,
        scale_factor: Option<f64>,
        snapping: Option<&str>,
    ) -> Result<String, String> {

        let flat_coords: Vec<f64> = coordinates.iter().flat_map(|&(lon, lat)| vec![lon, lat]).collect();
        let sources_vec = sources.unwrap_or(&[]).to_vec();
        let dests_vec = destinations.unwrap_or(&[]).to_vec();

        // Prepare bearings
        let bearings_flat: Vec<f64> = if let Some(b) = bearings {
            b.iter().flat_map(|&(value, range)| vec![value, range]).collect()
        } else {
            vec![]
        };

        // Prepare radiuses
        let radiuses_vec: Vec<f64> = radiuses.map(|r| r.to_vec()).unwrap_or_default();

        // Prepare hints as C strings
        let hints_cstrings: Vec<CString> = if let Some(h) = hints {
            h.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let hints_ptrs: Vec<*const c_char> = hints_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare approaches as C strings
        let approaches_cstrings: Vec<CString> = if let Some(a) = approaches {
            a.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let approaches_ptrs: Vec<*const c_char> = approaches_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare fallback_coordinate
        let fallback_coordinate_cstring = fallback_coordinate.and_then(|s| CString::new(s).ok());
        let fallback_coordinate_ptr = fallback_coordinate_cstring.as_ref().map(|cs| cs.as_ptr()).unwrap_or(std::ptr::null());

        // Prepare snapping
        let snapping_cstring = snapping.and_then(|s| CString::new(s).ok());
        let snapping_ptr = snapping_cstring.as_ref().map(|cs| cs.as_ptr()).unwrap_or(std::ptr::null());

        let result = unsafe {
            osrm_table(
                self.instance,
                flat_coords.as_ptr(),
                coordinates.len(),
                sources_vec.as_ptr(),
                sources_vec.len(),
                dests_vec.as_ptr(),
                dests_vec.len(),
                include_duration,
                include_distance,
                if bearings_flat.is_empty() { std::ptr::null() } else { bearings_flat.as_ptr() },
                bearings_flat.len() / 2,
                if radiuses_vec.is_empty() { std::ptr::null() } else { radiuses_vec.as_ptr() },
                radiuses_vec.len(),
                if hints_ptrs.is_empty() { std::ptr::null() } else { hints_ptrs.as_ptr() },
                hints_ptrs.len(),
                generate_hints,
                if approaches_ptrs.is_empty() { std::ptr::null() } else { approaches_ptrs.as_ptr() },
                approaches_ptrs.len(),
                fallback_speed.unwrap_or(-1.0),
                fallback_coordinate_ptr,
                scale_factor.unwrap_or(-1.0),
                snapping_ptr,
            )
        };

        let message_ptr = result.message;
        if message_ptr.is_null() {
            return Err("OSRM returned a null message".to_string());
        }

        let c_str = unsafe { CStr::from_ptr(message_ptr) };
        let rust_str = c_str.to_str().map_err(|e| e.to_string())?.to_owned();

        unsafe {
            osrm_free_string(message_ptr);
        }

        if result.code != 0 {
            return Err(format!("OSRM error: {}", rust_str));
        }

        Ok(rust_str)
    }

    pub(crate) fn match_route(
        &self,
        coordinates: &[(f64, f64)],
        timestamps: Option<&[u32]>,
        radiuses: Option<&[f64]>,
        bearings: Option<&[(f64, f64)]>,
        hints: Option<&[Option<String>]>,
        generate_hints: bool,
        approaches: Option<&[Option<String>]>,
        gaps: Option<&str>,
        tidy: bool,
        waypoints: Option<&[usize]>,
        snapping: Option<&str>,
    ) -> Result<String, String> {

        let flat_coords: Vec<f64> = coordinates.iter().flat_map(|&(lon, lat)| vec![lon, lat]).collect();

        // Prepare timestamps
        let timestamps_vec: Vec<u32> = timestamps.map(|t| t.to_vec()).unwrap_or_default();

        // Prepare radiuses
        let radiuses_vec: Vec<f64> = radiuses.map(|r| r.to_vec()).unwrap_or_default();

        // Prepare bearings
        let bearings_flat: Vec<f64> = if let Some(b) = bearings {
            b.iter().flat_map(|&(value, range)| vec![value, range]).collect()
        } else {
            vec![]
        };

        // Prepare hints as C strings
        let hints_cstrings: Vec<CString> = if let Some(h) = hints {
            h.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let hints_ptrs: Vec<*const c_char> = hints_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare approaches as C strings
        let approaches_cstrings: Vec<CString> = if let Some(a) = approaches {
            a.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let approaches_ptrs: Vec<*const c_char> = approaches_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare gaps
        let gaps_cstring = gaps.and_then(|s| CString::new(s).ok());
        let gaps_ptr = gaps_cstring.as_ref().map(|cs| cs.as_ptr()).unwrap_or(std::ptr::null());

        // Prepare waypoints
        let waypoints_vec: Vec<usize> = waypoints.map(|w| w.to_vec()).unwrap_or_default();

        // Prepare snapping
        let snapping_cstring = snapping.and_then(|s| CString::new(s).ok());
        let snapping_ptr = snapping_cstring.as_ref().map(|cs| cs.as_ptr()).unwrap_or(std::ptr::null());

        let result = unsafe {
            osrm_match(
                self.instance,
                flat_coords.as_ptr(),
                coordinates.len(),
                if timestamps_vec.is_empty() { std::ptr::null() } else { timestamps_vec.as_ptr() },
                timestamps_vec.len(),
                if radiuses_vec.is_empty() { std::ptr::null() } else { radiuses_vec.as_ptr() },
                radiuses_vec.len(),
                if bearings_flat.is_empty() { std::ptr::null() } else { bearings_flat.as_ptr() },
                bearings_flat.len() / 2,
                if hints_ptrs.is_empty() { std::ptr::null() } else { hints_ptrs.as_ptr() },
                hints_ptrs.len(),
                generate_hints,
                if approaches_ptrs.is_empty() { std::ptr::null() } else { approaches_ptrs.as_ptr() },
                approaches_ptrs.len(),
                gaps_ptr,
                tidy,
                if waypoints_vec.is_empty() { std::ptr::null() } else { waypoints_vec.as_ptr() },
                waypoints_vec.len(),
                snapping_ptr,
            )
        };

        let message_ptr = result.message;
        if message_ptr.is_null() {
            return Err("OSRM returned a null message".to_string());
        }

        let c_str = unsafe { CStr::from_ptr(message_ptr) };
        let rust_str = c_str.to_str().map_err(|e| e.to_string())?.to_owned();

        unsafe {
            osrm_free_string(message_ptr);
        }

        if result.code != 0 {
            return Err(format!("OSRM error: {}", rust_str));
        }

        Ok(rust_str)
    }

    pub(crate) fn nearest(
        &self,
        coordinate: (f64, f64),
        bearings: Option<&[(f64, f64)]>,
        radiuses: Option<&[f64]>,
        hints: Option<&[Option<String>]>,
        generate_hints: bool,
        number: Option<i32>,
        approaches: Option<&[Option<String>]>,
        snapping: Option<&str>,
    ) -> Result<String, String> {
        // Note: nearest only takes a single coordinate
        let flat_coords: Vec<f64> = vec![coordinate.0, coordinate.1];

        // Prepare bearings
        let bearings_flat: Vec<f64> = if let Some(b) = bearings {
            b.iter().flat_map(|&(value, range)| vec![value, range]).collect()
        } else {
            vec![]
        };

        // Prepare radiuses
        let radiuses_vec: Vec<f64> = radiuses.map(|r| r.to_vec()).unwrap_or_default();

        // Prepare hints as C strings
        let hints_cstrings: Vec<CString> = if let Some(h) = hints {
            h.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let hints_ptrs: Vec<*const c_char> = hints_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare approaches as C strings
        let approaches_cstrings: Vec<CString> = if let Some(a) = approaches {
            a.iter().filter_map(|opt| {
                opt.as_ref().and_then(|s| CString::new(s.as_str()).ok())
            }).collect()
        } else {
            vec![]
        };
        let approaches_ptrs: Vec<*const c_char> = approaches_cstrings.iter().map(|cs| cs.as_ptr()).collect();

        // Prepare snapping
        let snapping_cstring = snapping.and_then(|s| CString::new(s).ok());
        let snapping_ptr = snapping_cstring.as_ref().map(|cs| cs.as_ptr()).unwrap_or(std::ptr::null());

        let result = unsafe {
            osrm_nearest(
                self.instance,
                flat_coords.as_ptr(),
                1, // Only 1 coordinate for nearest
                if bearings_flat.is_empty() { std::ptr::null() } else { bearings_flat.as_ptr() },
                bearings_flat.len() / 2,
                if radiuses_vec.is_empty() { std::ptr::null() } else { radiuses_vec.as_ptr() },
                radiuses_vec.len(),
                if hints_ptrs.is_empty() { std::ptr::null() } else { hints_ptrs.as_ptr() },
                hints_ptrs.len(),
                generate_hints,
                number.unwrap_or(1),
                if approaches_ptrs.is_empty() { std::ptr::null() } else { approaches_ptrs.as_ptr() },
                approaches_ptrs.len(),
                snapping_ptr,
            )
        };

        let message_ptr = result.message;
        if message_ptr.is_null() {
            return Err("OSRM returned a null message".to_string());
        }

        let c_str = unsafe { CStr::from_ptr(message_ptr) };
        let rust_str = c_str.to_str().map_err(|e| e.to_string())?.to_owned();

        unsafe {
            osrm_free_string(message_ptr);
        }

        if result.code != 0 {
            return Err(format!("OSRM error: {}", rust_str));
        }

        Ok(rust_str)
    }
}

impl Drop for Osrm {
    fn drop(&mut self) {
        unsafe {
            osrm_destroy(self.instance);
        }
    }
}

unsafe impl Send for Osrm {}
unsafe impl Sync for Osrm {}
