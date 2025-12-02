use derive_builder::Builder;
use serde::{Deserialize, Serialize};
use crate::point::Point;
use crate::route::Route;
use crate::waypoints::Waypoint;

#[derive(Debug, Builder)]
pub struct TripRequest {
    pub points : Vec<Point>,
}

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct TripResponse {
    pub code: String,
    pub trips: Vec<Route>,
    pub waypoints: Vec<Waypoint>,
}
