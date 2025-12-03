// match.rs
use crate::point::Point;
use crate::route::RouteResponse;
use serde::{Deserialize, Serialize};

/// Request for map matching GPS traces to the road network
#[derive(Debug, Clone)]
pub struct MatchRequest {
    /// GPS coordinates to match
    pub points: Vec<Point>,
    /// Timestamps for each coordinate (UNIX-like timestamp in seconds)
    pub timestamps: Option<Vec<u32>>,
    /// Standard deviation of GPS precision in meters (use GPS accuracy if available)
    pub radiuses: Option<Vec<Option<f64>>>,
    /// Bearing and range constraints for each coordinate
    pub bearings: Option<Vec<Option<(i16, i16)>>>,
    /// Hints for coordinate snapping (base64 encoded strings)
    pub hints: Option<Vec<Option<String>>>,
    /// Whether to generate hints in response
    pub generate_hints: bool,
    /// Approach constraints for each coordinate
    pub approaches: Option<Vec<Option<String>>>,
    /// Gap handling: "split" (default) or "ignore"
    pub gaps: Option<String>,
    /// Allow track modification for better matching quality
    pub tidy: bool,
    /// Indices of coordinates to treat as waypoints (must include first and last)
    pub waypoints: Option<Vec<usize>>,
    /// Snapping type: "default" or "any"
    pub snapping: Option<String>,
}

/// Response from a map matching request
/// 
/// The response uses the same structure as RouteResponse, but with matchings
/// instead of routes, and tracepoints instead of waypoints
pub type MatchResponse = RouteResponse;
