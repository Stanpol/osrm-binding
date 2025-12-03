// osrm/src/lib.rs


use crate::errors::OsrmError;
use crate::{algorithm, Osrm, EngineConfig};
use crate::point::Point;
use crate::route::{RouteRequest, RouteResponse, SimpleRouteResponse};
use crate::tables::{TableRequest, TableResponse};
use crate::trip::{TripRequest, TripResponse};
use crate::r#match::{MatchRequest, MatchResponse};
use crate::nearest::{NearestRequest, NearestResponse};

pub struct OsrmEngine {
    instance: Osrm,
}

impl OsrmEngine {

    pub fn new(base_path: &str, algorithm : algorithm::Algorithm, max_table_size: Option<i32>) -> Result<Self, OsrmError> {
        let osrm = Osrm::new(base_path, algorithm.as_str(), max_table_size.unwrap_or(0)).map_err( |_|  OsrmError::Initialization )?;
        Ok(OsrmEngine {
            instance: osrm,
        })
    }

    pub fn new_with_config(config: EngineConfig) -> Result<Self, OsrmError> {
        let osrm = Osrm::new_with_config(config).map_err(|_| OsrmError::Initialization)?;
        Ok(OsrmEngine {
            instance: osrm,
        })
    }

    pub fn table(&self, table_request: TableRequest) -> Result<TableResponse, OsrmError> {
        let coordinates = &table_request.coordinates;
        let sources_index: Vec<usize> = table_request.sources_indices.unwrap_or_else(|| (0..coordinates.len()).collect());
        let destinations_index: Vec<usize> = table_request.destinations_indices.unwrap_or_else(|| (0..coordinates.len()).collect());
        
        // Prepare bearings
        let bearings_vec: Option<Vec<(f64, f64)>> = table_request.bearings.as_ref().map(|bearings| {
            bearings.iter().map(|b| {
                b.map(|(v, r)| (v as f64, r as f64)).unwrap_or((-1.0, -1.0))
            }).collect()
        });
        
        // Prepare radiuses
        let radiuses_vec: Option<Vec<f64>> = table_request.radiuses.as_ref().map(|radiuses| {
            radiuses.iter().map(|r| r.unwrap_or(-1.0)).collect()
        });
        
        // Prepare approaches
        let approaches_vec: Option<Vec<Option<String>>> = table_request.approaches.clone();
        
        let result = self.instance.table(
            coordinates,
            Some(&sources_index),
            Some(&destinations_index),
            table_request.include_duration,
            table_request.include_distance,
            bearings_vec.as_deref(),
            radiuses_vec.as_deref(),
            table_request.hints.as_deref(),
            table_request.generate_hints,
            approaches_vec.as_deref(),
            table_request.fallback_speed,
            table_request.fallback_coordinate.as_deref(),
            table_request.scale_factor,
            table_request.snapping.as_deref(),
        ).map_err(|e| OsrmError::FfiError(e))?;
        serde_json::from_str::<TableResponse>(&result).map_err(|e| OsrmError::JsonParse(e))
    }

    pub fn route(&self, route_request: RouteRequest) -> Result<RouteResponse, OsrmError> {
        let len = route_request.points.len();
        if len == 0 {
            return Err(OsrmError::InvalidTableArgument);
        }
        let coordinates: &[(f64, f64)] = &route_request.points.iter().map( |p|  (p.longitude, p.latitude) ).collect::<Vec<(f64, f64)>>()[..];
        let result = self.instance.route(coordinates).map_err( |e| OsrmError::FfiError(e))?;
        serde_json::from_str::<RouteResponse>(&result).map_err(|e| OsrmError::JsonParse(e))
    }

    pub fn trip(&self, trip_request: TripRequest) -> Result<TripResponse, OsrmError> {
        let len = trip_request.points.len();
        if len == 0 {
            return Err(OsrmError::InvalidTableArgument);
        }
        let coordinates: &[(f64, f64)] =  &trip_request.points.iter().map( |p|  (p.longitude, p.latitude) ).collect::<Vec<(f64, f64)>>()[..];
        let result = self.instance.trip(coordinates).map_err( |e| OsrmError::FfiError(e))?;
        serde_json::from_str::<TripResponse>(&result).map_err(|e| OsrmError::JsonParse(e))
    }

    pub fn simple_route(&self, from : Point , to : Point) -> Result<SimpleRouteResponse, OsrmError> {
        let coordinates: &[(f64, f64)] =  &[from, to].iter().map( |p |  (p.longitude, p.latitude)).collect::<Vec<(f64, f64)>>()[..];
        let result = self.instance.route(coordinates).map_err( |e| OsrmError::FfiError(e))?;
        let route_response = serde_json::from_str::<RouteResponse>(&result).map_err(|e| OsrmError::JsonParse(e))?;
        if route_response.routes.len() == 0 {
            return Err(OsrmError::ApiError("No route were returned between those 2 points".to_owned()))
        }
        Ok(SimpleRouteResponse {
            code: route_response.code,
            distance : route_response.routes.first().unwrap().legs.first().unwrap().distance,
            durations : route_response.routes.first().unwrap().legs.first().unwrap().duration
        })
    }

    pub fn match_route(&self, match_request: MatchRequest) -> Result<MatchResponse, OsrmError> {
        let len = match_request.points.len();
        if len == 0 {
            return Err(OsrmError::InvalidTableArgument);
        }
        
        let coordinates: Vec<(f64, f64)> = match_request.points.iter()
            .map(|p| (p.longitude, p.latitude))
            .collect();
        
        // Prepare bearings
        let bearings_vec: Option<Vec<(f64, f64)>> = match_request.bearings.as_ref().map(|bearings| {
            bearings.iter().map(|b| {
                b.map(|(v, r)| (v as f64, r as f64)).unwrap_or((-1.0, -1.0))
            }).collect()
        });
        
        // Prepare radiuses
        let radiuses_vec: Option<Vec<f64>> = match_request.radiuses.as_ref().map(|radiuses| {
            radiuses.iter().map(|r| r.unwrap_or(-1.0)).collect()
        });
        
        let result = self.instance.match_route(
            &coordinates,
            match_request.timestamps.as_deref(),
            radiuses_vec.as_deref(),
            bearings_vec.as_deref(),
            match_request.hints.as_deref(),
            match_request.generate_hints,
            match_request.approaches.as_deref(),
            match_request.gaps.as_deref(),
            match_request.tidy,
            match_request.waypoints.as_deref(),
            match_request.snapping.as_deref(),
        ).map_err(|e| OsrmError::FfiError(e))?;
        
        serde_json::from_str::<MatchResponse>(&result).map_err(|e| OsrmError::JsonParse(e))
    }

    pub fn nearest(&self, nearest_request: NearestRequest) -> Result<NearestResponse, OsrmError> {
        let coordinate = (nearest_request.coordinate.longitude, nearest_request.coordinate.latitude);
        
        // Prepare bearings
        let bearings_vec: Option<Vec<(f64, f64)>> = nearest_request.bearings.as_ref().map(|bearings| {
            bearings.iter().map(|b| {
                b.map(|(v, r)| (v as f64, r as f64)).unwrap_or((-1.0, -1.0))
            }).collect()
        });
        
        // Prepare radiuses
        let radiuses_vec: Option<Vec<f64>> = nearest_request.radiuses.as_ref().map(|radiuses| {
            radiuses.iter().map(|r| r.unwrap_or(-1.0)).collect()
        });
        
        // Prepare approaches
        let approaches_vec: Option<Vec<Option<String>>> = nearest_request.approaches.clone();
        
        let result = self.instance.nearest(
            coordinate,
            bearings_vec.as_deref(),
            radiuses_vec.as_deref(),
            nearest_request.hints.as_deref(),
            nearest_request.generate_hints,
            nearest_request.number,
            approaches_vec.as_deref(),
            nearest_request.snapping.as_deref(),
        ).map_err(|e| OsrmError::FfiError(e))?;
        
        serde_json::from_str::<NearestResponse>(&result).map_err(|e| OsrmError::JsonParse(e))
    }
}

#[cfg(test)]
mod tests {
    use super::*; // Import OsrmEngine, TableRequest, etc.
    use crate::algorithm::Algorithm;
    use crate::route::RouteRequestBuilder;
    use crate::tables::{Point};
    #[test]
    fn it_calculates_a_table_successfully() {
        // Try to load .env file, but don't fail if it doesn't exist
        let _ = dotenvy::dotenv();
        
        let path = match std::env::var("OSRM_TEST_DATA_PATH_MLD") {
            Ok(p) => p,
            Err(_) => {
                eprintln!("Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set");
                return;
            }
        };
        let engine = OsrmEngine::new(&*path, Algorithm::MLD, None).expect("Failed to initialize OSRM engine");

        let request = TableRequest {
            coordinates: vec![
                (6.1319, 49.6116), // Luxembourg City
                (6.1063, 49.7508), // Ettelbruck
                (5.9675, 49.5009), // Esch-sur-Alzette
            ],
            include_duration: true,
            include_distance: true,
            bearings: None,
            radiuses: None,
            hints: None,
            generate_hints: true,
            sources_indices: Some(vec![0]),
            destinations_indices: Some(vec![1, 2]),
            approaches: None,
            fallback_speed: None,
            fallback_coordinate: None,
            scale_factor: None,
            snapping: None,
        };
        let response = engine.table(request).expect("Table request failed");

        println!("{:?}", response.durations);

        assert_eq!(response.code, "Ok");
        let durations = response.durations.as_ref().expect("Durations should be present");
        assert_eq!(durations.len(), 1, "Should have 1 row for 1 source");
        assert_eq!(durations[0].len(), 2, "Should have 2 columns for 2 destinations");
        assert!(durations[0][0].is_some(), "Paris-Marseille duration should exist");
        assert!(durations[0][1].is_some(), "Paris-Lyon duration should exist");
    }

    #[test]
    fn it_calculates_a_route_successfully() {
        // Try to load .env file, but don't fail if it doesn't exist
        let _ = dotenvy::dotenv();
        
        let path = match std::env::var("OSRM_TEST_DATA_PATH_MLD") {
            Ok(p) => p,
            Err(_) => {
                eprintln!("Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set");
                return;
            }
        };
        let engine = OsrmEngine::new(&*path, Algorithm::MLD, None).expect("Failed to initialize OSRM engine");

        let request = RouteRequestBuilder::default().points(vec![Point { longitude: 6.1319, latitude: 49.6116 }, Point { longitude: 6.1063, latitude: 49.7508 }]).build().expect("Failed to build RouteRequest");
        let response = engine.route(request).expect("route request failed");

        let duration = response.routes.first().unwrap().legs.first().unwrap().duration;
        let distance = response.routes.first().unwrap().legs.first().unwrap().distance / 1000.0; // kilometer
        assert_eq!(response.code, "Ok");
        assert_eq!(response.routes.len(), 1, "Should have 1 row for 1 route");
        assert!(distance > 0.0, "Distance should be positive");
        assert!(duration > 0.0, "Duration should be positive");
        println!("Route: distance={:.2} km, duration={:.2} seconds", distance, duration);
    }

    #[test]
    fn it_calculates_a_simple_route_successfully() {
        // Try to load .env file, but don't fail if it doesn't exist
        let _ = dotenvy::dotenv();
        
        let path = match std::env::var("OSRM_TEST_DATA_PATH_MLD") {
            Ok(p) => p,
            Err(_) => {
                eprintln!("Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set");
                return;
            }
        };
        let engine = OsrmEngine::new(&*path, Algorithm::MLD, None).expect("Failed to initialize OSRM engine");
        let response = engine.simple_route(Point { longitude: 6.1319, latitude: 49.6116 }, Point { longitude: 6.1063, latitude: 49.7508 }).expect("route request failed");
        assert_eq!(response.code, "Ok");
        println!("Simple route: {:?}", response);
        assert!(response.distance > 0.0, "Distance should be positive");
        assert!(response.durations > 0.0, "Duration should be positive");
        println!("Simple route: distance={:.2} km, duration={:.2} seconds", response.distance / 1000.0, response.durations);
    }

    #[test]
    fn it_calculates_a_trip_successfully() {
        // Try to load .env file, but don't fail if it doesn't exist
        let _ = dotenvy::dotenv();
        
        let path = match std::env::var("OSRM_TEST_DATA_PATH_MLD") {
            Ok(p) => p,
            Err(_) => {
                eprintln!("Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set");
                return;
            }
        };
        let engine = OsrmEngine::new(&*path, Algorithm::MLD, None).expect("Failed to initialize OSRM engine");

        let request = crate::trip::TripRequest {
            points: vec![
                Point { longitude: 6.1319, latitude: 49.6116 }, // Luxembourg City
                Point { longitude: 6.1063, latitude: 49.7508 }, // Ettelbruck
                Point { longitude: 5.9675, latitude: 49.5009 }  // Esch-sur-Alzette
            ]
        };
        let response = engine.trip(request).expect("trip request failed");

        assert_eq!(response.code, "Ok");
        assert_eq!(response.trips.len(), 1, "Should have 1 trip");
        assert_eq!(response.waypoints.len(), 3, "Should have 3 waypoints");
        
        let trip = &response.trips[0];
        assert!(trip.distance > 0.0, "Trip should have positive distance");
        assert!(trip.duration > 0.0, "Trip should have positive duration");
        println!("Trip distance: {:.2} km, duration: {:.2} seconds", trip.distance / 1000.0, trip.duration);
    }
}
