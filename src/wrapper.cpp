// osrm_wrapper.cpp
#include <osrm/osrm.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <util/json_renderer.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/trip_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/nearest_parameters.hpp>

#include <string>
#include <iostream>
#include <cstdlib>
#include <cstring>

extern "C" {

    struct OSRM_Result {
        int code;
        char* message;
    };

    struct OSRM_Config {
        const char* algorithm;
        bool shared_memory;
        const char* dataset_name;
        bool mmap_memory;
        const char* path;
        int disable_feature_dataset_flags; // Bitfield: 1=ROUTE_STEPS, 2=ROUTE_GEOMETRY
        int max_locations_trip;
        int max_locations_viaroute;
        int max_locations_distance_table;
        int max_locations_map_matching;
        double max_radius_map_matching;
        int max_results_nearest;
        int max_alternatives;
        double default_radius;
    };

    void* osrm_create_with_config(const OSRM_Config* user_config) {
        try {
            osrm::EngineConfig config;
            
            // Set algorithm
            if (user_config->algorithm != nullptr) {
                if (strcmp(user_config->algorithm, "CH") == 0) {
                    config.algorithm = osrm::EngineConfig::Algorithm::CH;
                } else if (strcmp(user_config->algorithm, "MLD") == 0) {
                    config.algorithm = osrm::EngineConfig::Algorithm::MLD;
                } else {
                    config.algorithm = osrm::EngineConfig::Algorithm::MLD;
                }
            }
            
            // Set storage configuration
            config.use_shared_memory = user_config->shared_memory;
            
            // Note: memory_file is a path, not a boolean. If mmap_memory is true,
            // set it to the same path as storage_config
            if (user_config->dataset_name != nullptr && strlen(user_config->dataset_name) > 0) {
                config.dataset_name = std::string(user_config->dataset_name);
            }
            
            if (user_config->path != nullptr && strlen(user_config->path) > 0) {
                std::string path_str(user_config->path);
                config.storage_config = {path_str};
                
                // If mmap_memory is enabled, set memory_file to the path
                if (user_config->mmap_memory) {
                    config.memory_file = path_str;
                }
            }
            
            // Note: load_steps and load_geometry don't exist in OSRM v6.0.0
            // The disable_feature_dataset option is not supported in this version
            // These features may be available in newer OSRM versions
            
            // Set max locations
            if (user_config->max_locations_trip > 0) {
                config.max_locations_trip = user_config->max_locations_trip;
            }
            if (user_config->max_locations_viaroute > 0) {
                config.max_locations_viaroute = user_config->max_locations_viaroute;
            }
            if (user_config->max_locations_distance_table > 0) {
                config.max_locations_distance_table = user_config->max_locations_distance_table;
            }
            if (user_config->max_locations_map_matching > 0) {
                config.max_locations_map_matching = user_config->max_locations_map_matching;
            }
            
            // Set max radius and results
            if (user_config->max_radius_map_matching > 0) {
                config.max_radius_map_matching = user_config->max_radius_map_matching;
            }
            if (user_config->max_results_nearest > 0) {
                config.max_results_nearest = user_config->max_results_nearest;
            }
            if (user_config->max_alternatives > 0) {
                config.max_alternatives = user_config->max_alternatives;
            }
            if (user_config->default_radius > 0) {
                config.default_radius = user_config->default_radius;
            }

            return new osrm::OSRM(config);
        } catch (const std::exception& e) {
            std::cerr << "Fail to create an OSRM instance: " << e.what() << std::endl;
            return nullptr;
        }
    }

    // Backward compatibility function
    void* osrm_create(const char* base_path, const char* algorithm, int max_table_size) {
        OSRM_Config config = {};
        config.algorithm = algorithm;
        config.shared_memory = false;
        config.dataset_name = nullptr;
        config.mmap_memory = false;
        config.path = base_path;
        config.disable_feature_dataset_flags = 0;
        config.max_locations_trip = 0;
        config.max_locations_viaroute = 0;
        config.max_locations_distance_table = max_table_size;
        config.max_locations_map_matching = 0;
        config.max_radius_map_matching = 0;
        config.max_results_nearest = 0;
        config.max_alternatives = 0;
        config.default_radius = 0;
        
        return osrm_create_with_config(&config);
    }

    void osrm_destroy(void* osrm_instance) {
        if (osrm_instance) {
            delete static_cast<osrm::OSRM*>(osrm_instance);
        }
    }

    OSRM_Result osrm_table(void* osrm_instance,
                          const double* coordinates,
                          size_t num_coordinates,
                          const size_t* sources,
                          size_t num_sources,
                          const size_t* destinations,
                          size_t num_destinations,
                          bool include_duration,
                          bool include_distance,
                          const double* bearings,
                          size_t num_bearings,
                          const double* radiuses,
                          size_t num_radiuses,
                          const char** hints,
                          size_t num_hints,
                          bool generate_hints,
                          const char** approaches,
                          size_t num_approaches,
                          double fallback_speed,
                          const char* fallback_coordinate,
                          double scale_factor,
                          const char* snapping) {

        if (!osrm_instance) {
            const char* err = "OSRM instance not found";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        osrm::OSRM* osrm_ptr = static_cast<osrm::OSRM*>(osrm_instance);
        osrm::TableParameters params;

        for (size_t i = 0; i < num_coordinates; ++i) {
            params.coordinates.push_back({
                osrm::util::FloatLongitude{coordinates[i * 2]},
                osrm::util::FloatLatitude{coordinates[i * 2 + 1]}
            });
        }

        if (num_sources > 0) {
            params.sources.assign(sources, sources + num_sources);
        }

        if (num_destinations > 0) {
            params.destinations.assign(destinations, destinations + num_destinations);
        }

        // Set annotations based on the flags
        if (include_duration && include_distance) {
            params.annotations = osrm::TableParameters::AnnotationsType::All;
        } else if (include_duration) {
            params.annotations = osrm::TableParameters::AnnotationsType::Duration;
        } else if (include_distance) {
            params.annotations = osrm::TableParameters::AnnotationsType::Distance;
        } else {
            params.annotations = osrm::TableParameters::AnnotationsType::None;
        }

        // Set bearings
        if (num_bearings > 0 && bearings != nullptr) {
            for (size_t i = 0; i < num_bearings; ++i) {
                if (bearings[i * 2] >= 0) {  // Check for valid bearing
                    params.bearings.push_back(osrm::engine::Bearing{
                        static_cast<short>(bearings[i * 2]),
                        static_cast<short>(bearings[i * 2 + 1])
                    });
                } else {
                    params.bearings.push_back(std::nullopt);
                }
            }
        }

        // Set radiuses
        if (num_radiuses > 0 && radiuses != nullptr) {
            for (size_t i = 0; i < num_radiuses; ++i) {
                if (radiuses[i] >= 0) {
                    params.radiuses.push_back(radiuses[i]);
                } else {
                    params.radiuses.push_back(std::nullopt);
                }
            }
        }

        // Set hints
        if (num_hints > 0 && hints != nullptr) {
            for (size_t i = 0; i < num_hints; ++i) {
                if (hints[i] != nullptr && strlen(hints[i]) > 0) {
                    params.hints.push_back(osrm::engine::Hint::FromBase64(hints[i]));
                } else {
                    params.hints.push_back(std::nullopt);
                }
            }
        }

        // Set generate_hints
        params.generate_hints = generate_hints;

        // Set approaches
        if (num_approaches > 0 && approaches != nullptr) {
            for (size_t i = 0; i < num_approaches; ++i) {
                if (approaches[i] != nullptr) {
                    std::string approach_str(approaches[i]);
                    if (approach_str == "curb") {
                        params.approaches.push_back(osrm::engine::Approach::CURB);
                    } else if (approach_str == "opposite") {
                        params.approaches.push_back(osrm::engine::Approach::OPPOSITE);
                    } else {
                        params.approaches.push_back(osrm::engine::Approach::UNRESTRICTED);
                    }
                } else {
                    params.approaches.push_back(std::nullopt);
                }
            }
        }

        // Set fallback_speed
        if (fallback_speed > 0) {
            params.fallback_speed = fallback_speed;
        }

        // Set fallback_coordinate
        if (fallback_coordinate != nullptr) {
            std::string fallback_str(fallback_coordinate);
            if (fallback_str == "snapped") {
                params.fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Snapped;
            } else {
                params.fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Input;
            }
        }

        // Set scale_factor
        if (scale_factor > 0) {
            params.scale_factor = scale_factor;
        }

        // Set snapping
        if (snapping != nullptr) {
            std::string snapping_str(snapping);
            if (snapping_str == "any") {
                params.snapping = osrm::TableParameters::SnappingType::Any;
            } else {
                params.snapping = osrm::TableParameters::SnappingType::Default;
            }
        }

        osrm::json::Object result;
        const auto status = osrm_ptr->Table(params, result);

        std::string result_str;
        int code;

        if (status == osrm::Status::Ok) {
            code = 0;
            osrm::util::json::render(result_str, result);
        } else {
            code = 1;
            try {
                result_str = std::get<osrm::util::json::String>(result.values.at("message")).value;
            } catch (const std::exception& e) {
                result_str = "Unknown OSRM error";
            }
        }

    char* message = new char[result_str.length() + 1];
    strcpy(message, result_str.c_str());

    return {code, message};
    }

    OSRM_Result osrm_route(void* osrm_instance,
                           const double* coordinates,
                           size_t num_coordinates,
                           const double* bearings,
                           size_t num_bearings,
                           const double* radiuses,
                           size_t num_radiuses,
                           const char* const* hints,
                           size_t num_hints,
                           bool generate_hints,
                           const char* const* approaches,
                           size_t num_approaches,
                           const char* snapping,
                           bool steps,
                           int alternatives,
                           const char* const* annotations,
                           size_t num_annotations,
                           const char* geometries,
                           const char* overview,
                           bool continue_straight,
                           const char* const* exclude,
                           size_t num_exclude,
                           const size_t* waypoints,
                           size_t num_waypoints)
    {
        if (!osrm_instance) {
            const char* err = "OSRM instance not found";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        osrm::OSRM* osrm_ptr = static_cast<osrm::OSRM*>(osrm_instance);
        osrm::RouteParameters params;

        for (size_t i = 0; i < num_coordinates; ++i) {
            params.coordinates.push_back({
                osrm::util::FloatLongitude{coordinates[i * 2]},
                osrm::util::FloatLatitude{coordinates[i * 2 + 1]}
            });
        }

        // Set bearings - must match coordinates count if provided
        if (num_bearings > 0 && bearings != nullptr) {
            if (num_bearings == num_coordinates) {
                for (size_t i = 0; i < num_bearings; ++i) {
                    if (bearings[i * 2] >= 0) {
                        params.bearings.push_back(osrm::engine::Bearing{
                            static_cast<short>(bearings[i * 2]),
                            static_cast<short>(bearings[i * 2 + 1])
                        });
                    } else {
                        params.bearings.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set radiuses - must match coordinates count if provided
        if (num_radiuses > 0 && radiuses != nullptr) {
            if (num_radiuses == num_coordinates) {
                for (size_t i = 0; i < num_radiuses; ++i) {
                    if (radiuses[i] >= 0) {
                        params.radiuses.push_back(radiuses[i]);
                    } else {
                        params.radiuses.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set hints - must match coordinates count if provided
        if (num_hints > 0 && hints != nullptr) {
            if (num_hints == num_coordinates) {
                for (size_t i = 0; i < num_hints; ++i) {
                    if (hints[i] != nullptr && strlen(hints[i]) > 0) {
                        params.hints.push_back(osrm::engine::Hint::FromBase64(hints[i]));
                    } else {
                        params.hints.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set generate_hints
        params.generate_hints = generate_hints;

        // Set approaches - must match coordinates count if provided
        if (num_approaches > 0 && approaches != nullptr) {
            if (num_approaches == num_coordinates) {
                for (size_t i = 0; i < num_approaches; ++i) {
                    if (approaches[i] != nullptr) {
                        std::string approach_str(approaches[i]);
                        if (approach_str == "curb") {
                            params.approaches.push_back(osrm::engine::Approach::CURB);
                        } else if (approach_str == "opposite") {
                            params.approaches.push_back(osrm::engine::Approach::OPPOSITE);
                        } else {
                            params.approaches.push_back(osrm::engine::Approach::UNRESTRICTED);
                        }
                    } else {
                        params.approaches.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set snapping
        if (snapping != nullptr) {
            std::string snapping_str(snapping);
            if (snapping_str == "any") {
                params.snapping = osrm::RouteParameters::SnappingType::Any;
            } else {
                params.snapping = osrm::RouteParameters::SnappingType::Default;
            }
        }

        // Set steps
        params.steps = steps;

        // Set alternatives
        if (alternatives > 0) {
            params.alternatives = true;
            params.number_of_alternatives = alternatives;
        } else {
            params.alternatives = false;
        }

        // Set annotations
        if (num_annotations > 0 && annotations != nullptr) {
            params.annotations = true;
            params.annotations_type = osrm::RouteParameters::AnnotationsType::None;
            
            for (size_t i = 0; i < num_annotations; ++i) {
                if (annotations[i] != nullptr) {
                    std::string annotation_str(annotations[i]);
                    if (annotation_str == "true" || annotation_str == "all") {
                        params.annotations_type = osrm::RouteParameters::AnnotationsType::All;
                        break;
                    } else if (annotation_str == "nodes") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Nodes));
                    } else if (annotation_str == "distance") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Distance));
                    } else if (annotation_str == "duration") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Duration));
                    } else if (annotation_str == "datasources") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Datasources));
                    } else if (annotation_str == "weight") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Weight));
                    } else if (annotation_str == "speed") {
                        params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Speed));
                    }
                }
            }
        } else {
            params.annotations = false;
        }

        // Set geometries
        if (geometries != nullptr) {
            std::string geometries_str(geometries);
            if (geometries_str == "polyline") {
                params.geometries = osrm::RouteParameters::GeometriesType::Polyline;
            } else if (geometries_str == "polyline6") {
                params.geometries = osrm::RouteParameters::GeometriesType::Polyline6;
            } else if (geometries_str == "geojson") {
                params.geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
            } else {
                params.geometries = osrm::RouteParameters::GeometriesType::Polyline;
            }
        }

        // Set overview
        if (overview != nullptr) {
            std::string overview_str(overview);
            if (overview_str == "simplified") {
                params.overview = osrm::RouteParameters::OverviewType::Simplified;
            } else if (overview_str == "full") {
                params.overview = osrm::RouteParameters::OverviewType::Full;
            } else if (overview_str == "false") {
                params.overview = osrm::RouteParameters::OverviewType::False;
            } else {
                params.overview = osrm::RouteParameters::OverviewType::Simplified;
            }
        }

        // Set continue_straight
        params.continue_straight = continue_straight;

        // Set exclude
        if (num_exclude > 0 && exclude != nullptr) {
            for (size_t i = 0; i < num_exclude; ++i) {
                if (exclude[i] != nullptr) {
                    params.exclude.push_back(std::string(exclude[i]));
                }
            }
        }

        // Set waypoints
        if (num_waypoints > 0 && waypoints != nullptr) {
            for (size_t i = 0; i < num_waypoints; ++i) {
                params.waypoints.push_back(waypoints[i]);
            }
        }

        osrm::json::Object result;
        const auto status = osrm_ptr->Route(params, result);

        std::string result_str;
        int code;

        if (status == osrm::Status::Ok) {
            code = 0;
            osrm::util::json::render(result_str, result);
        } else {
            code = 1;
            try {
                result_str = std::get<osrm::util::json::String>(result.values.at("message")).value;
            } catch (const std::exception& e) {
                result_str = "Unknown OSRM error";
            }
        }

        char* message = new char[result_str.length() + 1];
        strcpy(message, result_str.c_str());

        return {code, message};
    }

    OSRM_Result osrm_trip(void* osrm_instance,
                          const double* coordinates,
                          size_t num_coordinates,
                          const double* bearings,
                          size_t num_bearings,
                          const double* radiuses,
                          size_t num_radiuses,
                          const char* const* hints,
                          size_t num_hints,
                          bool generate_hints,
                          const char* const* approaches,
                          size_t num_approaches,
                          const char* snapping)
    {

            if (!osrm_instance) {
                const char* err = "OSRM instance not found";
                char* msg = new char[strlen(err) + 1];
                strcpy(msg, err);
                return {1, msg};
            }

            osrm::OSRM* osrm_ptr = static_cast<osrm::OSRM*>(osrm_instance);
            osrm::TripParameters params;

            for (size_t i = 0; i < num_coordinates; ++i) {
                params.coordinates.push_back({
                    osrm::util::FloatLongitude{coordinates[i * 2]},
                    osrm::util::FloatLatitude{coordinates[i * 2 + 1]}
                });
            }

            // Set bearings - must match coordinates count if provided
            if (num_bearings > 0 && bearings != nullptr) {
                if (num_bearings == num_coordinates) {
                    for (size_t i = 0; i < num_bearings; ++i) {
                        if (bearings[i * 2] >= 0) {
                            params.bearings.push_back(osrm::engine::Bearing{
                                static_cast<short>(bearings[i * 2]),
                                static_cast<short>(bearings[i * 2 + 1])
                            });
                        } else {
                            params.bearings.push_back(std::nullopt);
                        }
                    }
                }
            }

            // Set radiuses - must match coordinates count if provided
            if (num_radiuses > 0 && radiuses != nullptr) {
                if (num_radiuses == num_coordinates) {
                    for (size_t i = 0; i < num_radiuses; ++i) {
                        if (radiuses[i] >= 0) {
                            params.radiuses.push_back(radiuses[i]);
                        } else {
                            params.radiuses.push_back(std::nullopt);
                        }
                    }
                }
            }

            // Set hints - must match coordinates count if provided
            if (num_hints > 0 && hints != nullptr) {
                if (num_hints == num_coordinates) {
                    for (size_t i = 0; i < num_hints; ++i) {
                        if (hints[i] != nullptr && strlen(hints[i]) > 0) {
                            params.hints.push_back(osrm::engine::Hint::FromBase64(hints[i]));
                        } else {
                            params.hints.push_back(std::nullopt);
                        }
                    }
                }
            }

            // Set generate_hints
            params.generate_hints = generate_hints;

            // Set approaches - must match coordinates count if provided
            if (num_approaches > 0 && approaches != nullptr) {
                if (num_approaches == num_coordinates) {
                    for (size_t i = 0; i < num_approaches; ++i) {
                        if (approaches[i] != nullptr) {
                            std::string approach_str(approaches[i]);
                            if (approach_str == "curb") {
                                params.approaches.push_back(osrm::engine::Approach::CURB);
                            } else if (approach_str == "opposite") {
                                params.approaches.push_back(osrm::engine::Approach::OPPOSITE);
                            } else {
                                params.approaches.push_back(osrm::engine::Approach::UNRESTRICTED);
                            }
                        } else {
                            params.approaches.push_back(std::nullopt);
                        }
                    }
                }
            }

            // Set snapping
            if (snapping != nullptr) {
                std::string snapping_str(snapping);
                if (snapping_str == "any") {
                    params.snapping = osrm::TripParameters::SnappingType::Any;
                } else {
                    params.snapping = osrm::TripParameters::SnappingType::Default;
                }
            }

            osrm::json::Object result;
            const auto status = osrm_ptr->Trip(params, result);

            std::string result_str;
            int code;

            if (status == osrm::Status::Ok) {
                code = 0;
                osrm::util::json::render(result_str, result);
            } else {
                code = 1;
                try {
                    result_str = std::get<osrm::util::json::String>(result.values.at("message")).value;
                } catch (const std::exception& e) {
                    result_str = "Unknown OSRM error";
                }
            }

            char* message = new char[result_str.length() + 1];
            strcpy(message, result_str.c_str());

            return {code, message};
        }

    OSRM_Result osrm_match(void* osrm_instance,
                           const double* coordinates,
                           size_t num_coordinates,
                           const unsigned* timestamps,
                           size_t num_timestamps,
                           const double* radiuses,
                           size_t num_radiuses,
                           const double* bearings,
                           size_t num_bearings,
                           const char* const* hints,
                           size_t num_hints,
                           bool generate_hints,
                           const char* const* approaches,
                           size_t num_approaches,
                           const char* gaps,
                           bool tidy,
                           const size_t* waypoints,
                           size_t num_waypoints,
                           const char* snapping)
    {
        if (!osrm_instance) {
            const char* err = "OSRM instance not found";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        osrm::OSRM* osrm_ptr = static_cast<osrm::OSRM*>(osrm_instance);
        osrm::MatchParameters params;

        // Set coordinates
        for (size_t i = 0; i < num_coordinates; ++i) {
            params.coordinates.push_back({
                osrm::util::FloatLongitude{coordinates[i * 2]},
                osrm::util::FloatLatitude{coordinates[i * 2 + 1]}
            });
        }

        // Set timestamps
        if (num_timestamps > 0 && timestamps != nullptr) {
            for (size_t i = 0; i < num_timestamps; ++i) {
                params.timestamps.push_back(timestamps[i]);
            }
        }

        // Set radiuses - must match coordinates count if provided
        if (num_radiuses > 0 && radiuses != nullptr) {
            // Ensure we have the right count
            if (num_radiuses == num_coordinates) {
                for (size_t i = 0; i < num_radiuses; ++i) {
                    if (radiuses[i] >= 0) {
                        params.radiuses.push_back(radiuses[i]);
                    } else {
                        params.radiuses.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set bearings - must match coordinates count if provided
        if (num_bearings > 0 && bearings != nullptr) {
            // Ensure we have the right count
            if (num_bearings == num_coordinates) {
                for (size_t i = 0; i < num_bearings; ++i) {
                    if (bearings[i * 2] >= 0) {
                        params.bearings.push_back(osrm::engine::Bearing{
                            static_cast<short>(bearings[i * 2]),
                            static_cast<short>(bearings[i * 2 + 1])
                        });
                    } else {
                        params.bearings.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set hints - must match coordinates count if provided
        if (num_hints > 0 && hints != nullptr) {
            // Ensure we have the right count
            if (num_hints == num_coordinates) {
                for (size_t i = 0; i < num_hints; ++i) {
                    if (hints[i] != nullptr && strlen(hints[i]) > 0) {
                        params.hints.push_back(osrm::engine::Hint::FromBase64(hints[i]));
                    } else {
                        params.hints.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set generate_hints
        params.generate_hints = generate_hints;

        // Set approaches - must match coordinates count if provided
        if (num_approaches > 0 && approaches != nullptr) {
            // Ensure we have the right count
            if (num_approaches == num_coordinates) {
                for (size_t i = 0; i < num_approaches; ++i) {
                    if (approaches[i] != nullptr) {
                        std::string approach_str(approaches[i]);
                        if (approach_str == "curb") {
                            params.approaches.push_back(osrm::engine::Approach::CURB);
                        } else if (approach_str == "opposite") {
                            params.approaches.push_back(osrm::engine::Approach::OPPOSITE);
                        } else {
                            params.approaches.push_back(osrm::engine::Approach::UNRESTRICTED);
                        }
                    } else {
                        params.approaches.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set gaps (split or ignore)
        if (gaps != nullptr) {
            std::string gaps_str(gaps);
            if (gaps_str == "ignore") {
                params.gaps = osrm::MatchParameters::GapsType::Ignore;
            } else {
                params.gaps = osrm::MatchParameters::GapsType::Split;
            }
        }

        // Set tidy
        params.tidy = tidy;

        // Set waypoints
        if (num_waypoints > 0 && waypoints != nullptr) {
            for (size_t i = 0; i < num_waypoints; ++i) {
                params.waypoints.push_back(waypoints[i]);
            }
        }

        // Set snapping
        if (snapping != nullptr) {
            std::string snapping_str(snapping);
            if (snapping_str == "any") {
                params.snapping = osrm::MatchParameters::SnappingType::Any;
            } else {
                params.snapping = osrm::MatchParameters::SnappingType::Default;
            }
        }

        osrm::json::Object result;
        const auto status = osrm_ptr->Match(params, result);

        std::string result_str;
        int code;

        if (status == osrm::Status::Ok) {
            code = 0;
            osrm::util::json::render(result_str, result);
        } else {
            code = 1;
            try {
                result_str = std::get<osrm::util::json::String>(result.values.at("message")).value;
            } catch (const std::exception& e) {
                result_str = "Unknown OSRM error";
            }
        }

        char* message = new char[result_str.length() + 1];
        strcpy(message, result_str.c_str());

        return {code, message};
    }

    OSRM_Result osrm_nearest(
        void* osrm_instance,
        const double* coordinates,
        size_t num_coordinates,
        const double* bearings,
        size_t num_bearings,
        const double* radiuses,
        size_t num_radiuses,
        const char* const* hints,
        size_t num_hints,
        bool generate_hints,
        int number,
        const char* const* approaches,
        size_t num_approaches,
        const char* snapping
    ) {
        if (!osrm_instance) {
            const char* err = "OSRM instance not found";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        osrm::OSRM* osrm_ptr = static_cast<osrm::OSRM*>(osrm_instance);
        osrm::NearestParameters params;

        // Set coordinates (should be exactly 1 for nearest)
        for (size_t i = 0; i < num_coordinates; ++i) {
            params.coordinates.push_back({
                osrm::util::FloatLongitude{coordinates[i * 2]},
                osrm::util::FloatLatitude{coordinates[i * 2 + 1]}
            });
        }

        // Set number of results
        if (number > 0) {
            params.number_of_results = number;
        }

        // Set radiuses - must match coordinates count if provided
        if (num_radiuses > 0 && radiuses != nullptr) {
            if (num_radiuses == num_coordinates) {
                for (size_t i = 0; i < num_radiuses; ++i) {
                    if (radiuses[i] >= 0) {
                        params.radiuses.push_back(radiuses[i]);
                    } else {
                        params.radiuses.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set bearings - must match coordinates count if provided
        if (num_bearings > 0 && bearings != nullptr) {
            if (num_bearings == num_coordinates) {
                for (size_t i = 0; i < num_bearings; ++i) {
                    if (bearings[i * 2] >= 0) {
                        params.bearings.push_back(osrm::engine::Bearing{
                            static_cast<short>(bearings[i * 2]),
                            static_cast<short>(bearings[i * 2 + 1])
                        });
                    } else {
                        params.bearings.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set hints - must match coordinates count if provided
        if (num_hints > 0 && hints != nullptr) {
            if (num_hints == num_coordinates) {
                for (size_t i = 0; i < num_hints; ++i) {
                    if (hints[i] != nullptr && strlen(hints[i]) > 0) {
                        params.hints.push_back(osrm::engine::Hint::FromBase64(hints[i]));
                    } else {
                        params.hints.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set generate_hints
        params.generate_hints = generate_hints;

        // Set approaches - must match coordinates count if provided
        if (num_approaches > 0 && approaches != nullptr) {
            if (num_approaches == num_coordinates) {
                for (size_t i = 0; i < num_approaches; ++i) {
                    if (approaches[i] != nullptr) {
                        std::string approach_str(approaches[i]);
                        if (approach_str == "curb") {
                            params.approaches.push_back(osrm::engine::Approach::CURB);
                        } else if (approach_str == "opposite") {
                            params.approaches.push_back(osrm::engine::Approach::OPPOSITE);
                        } else {
                            params.approaches.push_back(osrm::engine::Approach::UNRESTRICTED);
                        }
                    } else {
                        params.approaches.push_back(std::nullopt);
                    }
                }
            }
        }

        // Set snapping
        if (snapping != nullptr) {
            std::string snapping_str(snapping);
            if (snapping_str == "any") {
                params.snapping = osrm::NearestParameters::SnappingType::Any;
            } else {
                params.snapping = osrm::NearestParameters::SnappingType::Default;
            }
        }

        osrm::json::Object result;
        const auto status = osrm_ptr->Nearest(params, result);

        std::string result_str;
        int code;

        if (status == osrm::Status::Ok) {
            code = 0;
            osrm::util::json::render(result_str, result);
        } else {
            code = 1;
            try {
                result_str = std::get<osrm::util::json::String>(result.values.at("message")).value;
            } catch (const std::exception& e) {
                result_str = "Unknown OSRM error";
            }
        }

        char* message = new char[result_str.length() + 1];
        strcpy(message, result_str.c_str());

        return {code, message};
    }

    void osrm_free_string(char* s) {
        if (s) {
            delete[] s;
        }
    }
}
