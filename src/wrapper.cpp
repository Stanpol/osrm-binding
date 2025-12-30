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
#include <contractor/contractor.hpp>
#include <contractor/contractor_config.hpp>
#include <customizer/customizer.hpp>
#include <customizer/customizer_config.hpp>
#include <partitioner/partitioner.hpp>
#include <partitioner/partitioner_config.hpp>
#include <storage/io_config.hpp>
#include <extractor/extractor.hpp>
#include <extractor/extractor_config.hpp>
#include <extractor/scripting_environment_lua.hpp>

#include <string>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <thread>

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
                           size_t num_waypoints,
                           bool skip_waypoints)
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

        // Set skip_waypoints
        params.skip_waypoints = skip_waypoints;

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
                          const char* snapping,
                          bool roundtrip,
                          const char* source,
                          const char* destination,
                          bool steps,
                          const char* const* annotations,
                          size_t num_annotations,
                          const char* geometries,
                          const char* overview,
                          const char* const* exclude,
                          size_t num_exclude)
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

            // Set roundtrip
            params.roundtrip = roundtrip;

            // Set source
            if (source != nullptr) {
                std::string source_str(source);
                if (source_str == "any") {
                    params.source = osrm::TripParameters::SourceType::Any;
                } else if (source_str == "first") {
                    params.source = osrm::TripParameters::SourceType::First;
                }
            }

            // Set destination
            if (destination != nullptr) {
                std::string destination_str(destination);
                if (destination_str == "any") {
                    params.destination = osrm::TripParameters::DestinationType::Any;
                } else if (destination_str == "last") {
                    params.destination = osrm::TripParameters::DestinationType::Last;
                }
            }

            // Set steps
            params.steps = steps;

            // Set annotations
            if (num_annotations > 0 && annotations != nullptr) {
                params.annotations = true;
                params.annotations_type = osrm::RouteParameters::AnnotationsType::None;
                
                for (size_t i = 0; i < num_annotations; ++i) {
                    if (annotations[i] != nullptr) {
                        std::string annotation(annotations[i]);
                        if (annotation == "true" || annotation == "all") {
                            params.annotations_type = osrm::RouteParameters::AnnotationsType::All;
                            break;
                        } else if (annotation == "duration") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Duration));
                        } else if (annotation == "distance") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Distance));
                        } else if (annotation == "speed") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Speed));
                        } else if (annotation == "weight") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Weight));
                        } else if (annotation == "datasources") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Datasources));
                        } else if (annotation == "nodes") {
                            params.annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(
                                static_cast<int>(params.annotations_type) | static_cast<int>(osrm::RouteParameters::AnnotationsType::Nodes));
                        }
                    }
                }
            } else {
                params.annotations = false;
            }

            // Set geometries
            if (geometries != nullptr) {
                std::string geom_str(geometries);
                if (geom_str == "polyline") {
                    params.geometries = osrm::RouteParameters::GeometriesType::Polyline;
                } else if (geom_str == "polyline6") {
                    params.geometries = osrm::RouteParameters::GeometriesType::Polyline6;
                } else if (geom_str == "geojson") {
                    params.geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
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
                }
            }

            // Set exclude
            if (num_exclude > 0 && exclude != nullptr) {
                for (size_t i = 0; i < num_exclude; ++i) {
                    if (exclude[i] != nullptr) {
                        params.exclude.push_back(std::string(exclude[i]));
                    }
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
                           const char* snapping,
                           bool steps,
                           const char* const* annotations,
                           size_t num_annotations,
                           const char* geometries,
                           const char* overview,
                           const char* const* exclude,
                           size_t num_exclude)
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

        // Set steps
        params.steps = steps;

        // Set annotations
        if (num_annotations > 0 && annotations != nullptr) {
            params.annotations = true;
            params.annotations_type = osrm::MatchParameters::AnnotationsType::None;
            
            for (size_t i = 0; i < num_annotations; ++i) {
                if (annotations[i] != nullptr) {
                    std::string annotation_str(annotations[i]);
                    if (annotation_str == "true" || annotation_str == "all") {
                        params.annotations_type = osrm::MatchParameters::AnnotationsType::All;
                        break;
                    } else if (annotation_str == "nodes") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Nodes));
                    } else if (annotation_str == "distance") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Distance));
                    } else if (annotation_str == "duration") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Duration));
                    } else if (annotation_str == "datasources") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Datasources));
                    } else if (annotation_str == "weight") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Weight));
                    } else if (annotation_str == "speed") {
                        params.annotations_type = static_cast<osrm::MatchParameters::AnnotationsType>(
                            static_cast<int>(params.annotations_type) | static_cast<int>(osrm::MatchParameters::AnnotationsType::Speed));
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
                params.geometries = osrm::MatchParameters::GeometriesType::Polyline;
            } else if (geometries_str == "polyline6") {
                params.geometries = osrm::MatchParameters::GeometriesType::Polyline6;
            } else if (geometries_str == "geojson") {
                params.geometries = osrm::MatchParameters::GeometriesType::GeoJSON;
            } else {
                params.geometries = osrm::MatchParameters::GeometriesType::Polyline;
            }
        }

        // Set overview
        if (overview != nullptr) {
            std::string overview_str(overview);
            if (overview_str == "simplified") {
                params.overview = osrm::MatchParameters::OverviewType::Simplified;
            } else if (overview_str == "full") {
                params.overview = osrm::MatchParameters::OverviewType::Full;
            } else if (overview_str == "false") {
                params.overview = osrm::MatchParameters::OverviewType::False;
            } else {
                params.overview = osrm::MatchParameters::OverviewType::Simplified;
            }
        }

        // Set exclude
        if (num_exclude > 0 && exclude != nullptr) {
            for (size_t i = 0; i < num_exclude; ++i) {
                if (exclude[i] != nullptr) {
                    params.exclude.push_back(std::string(exclude[i]));
                }
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

    OSRM_Result osrm_run_contract(const char* base_path, int threads) {
        if (!base_path) {
            const char* err = "Path cannot be null";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        try {
            osrm::contractor::ContractorConfig config;
            config.base_path = std::filesystem::path(base_path);
            config.UseDefaultOutputNames(config.base_path);
            
            // Set thread count: use provided value if > 0, otherwise use hardware concurrency
            if (threads > 0) {
                config.requested_num_threads = threads;
            } else {
                config.requested_num_threads = std::thread::hardware_concurrency();
            }

            osrm::contractor::Contractor contractor(config);
            int ret = contractor.Run();

            if (ret != 0) {
                const char* err = "Contractor run returned non-zero code";
                char* msg = new char[strlen(err) + 1];
                strcpy(msg, err);
                return {ret, msg};
            }

            // Success
            const char* success = "Contraction successful";
            char* msg = new char[strlen(success) + 1];
            strcpy(msg, success);
            return {0, msg};

        } catch (const std::exception& e) {
            std::string what = e.what();
            char* msg = new char[what.length() + 1];
            strcpy(msg, what.c_str());
            return {1, msg};
        }
    }

    OSRM_Result osrm_run_extract(
        const char* input_path,
        const char* profile_path,
        int threads,
        bool generate_edge_based_graph,
        bool generate_node_based_graph,
        bool parse_conditionals,
        bool use_metadata,
        bool use_locations_cache
    ) {
        if (!input_path) {
            const char* err = "Input path cannot be null";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }
        if (!profile_path) {
            const char* err = "Profile path cannot be null";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        try {
            osrm::extractor::ExtractorConfig config;
            config.input_path = std::filesystem::path(input_path);
            config.profile_path = std::filesystem::path(profile_path);
            
            // Set up output file names based on input path
            config.UseDefaultOutputNames(config.input_path);
            
            // Set thread count: use provided value if > 0, otherwise use hardware concurrency
            if (threads > 0) {
                config.requested_num_threads = threads;
            } else {
                config.requested_num_threads = std::thread::hardware_concurrency();
            }
            
            config.use_metadata = use_metadata;
            config.use_locations_cache = use_locations_cache;
            config.parse_conditionals = parse_conditionals;
            
            // Note: OSRM's extractor writes multiple files (.osrm, .osrm.names, etc.)
            // The logic inside Extractor::run handles these based on input_path.

            osrm::extractor::Extractor extractor(config);
            // Extractor needs a ScriptingEnvironment
            osrm::extractor::Sol2ScriptingEnvironment scripting_environment(
                config.profile_path.string(), 
                config.location_dependent_data_paths
            );

            int ret = extractor.run(scripting_environment);

            if (ret != 0) {
                const char* err = "Extraction run returned non-zero code";
                char* msg = new char[strlen(err) + 1];
                strcpy(msg, err);
                return {ret, msg};
            }

            const char* success = "Extraction successful";
            char* msg = new char[strlen(success) + 1];
            strcpy(msg, success);
            return {0, msg};

        } catch (const std::exception& e) {
            std::string what = e.what();
            char* msg = new char[what.length() + 1];
            strcpy(msg, what.c_str());
            return {1, msg};
        }
    }

    OSRM_Result osrm_run_partition(
        const char* base_path, 
        int threads,
        double balance,
        double boundary_factor,
        int num_optimizing_cuts,
        int small_component_size,
        const int* max_cell_sizes,
        size_t num_max_cell_sizes
    ) {
        if (!base_path) {
            const char* err = "Path cannot be null";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        try {
            osrm::partitioner::PartitionerConfig config;
            config.base_path = std::filesystem::path(base_path);
            config.UseDefaultOutputNames(config.base_path);
            
            // Set thread count: use provided value if > 0, otherwise use hardware concurrency
            if (threads > 0) {
                config.requested_num_threads = threads;
            } else {
                config.requested_num_threads = std::thread::hardware_concurrency();
            }
            
            // Apply overrides if values are valid
            if (balance > 0) config.balance = balance;
            if (boundary_factor > 0) config.boundary_factor = boundary_factor;
            if (num_optimizing_cuts > 0) config.num_optimizing_cuts = num_optimizing_cuts;
            if (small_component_size > 0) config.small_component_size = small_component_size;
            
            // If cell sizes are provided, override the defaults
            if (max_cell_sizes != nullptr && num_max_cell_sizes > 0) {
                config.max_cell_sizes.clear();
                for (size_t i = 0; i < num_max_cell_sizes; ++i) {
                    config.max_cell_sizes.push_back(static_cast<std::size_t>(max_cell_sizes[i]));
                }
            }

            osrm::partitioner::Partitioner partitioner;
            int ret = partitioner.Run(config);

            if (ret != 0) {
                const char* err = "Partition run returned non-zero code";
                char* msg = new char[strlen(err) + 1];
                strcpy(msg, err);
                return {ret, msg};
            }

            const char* success = "Partitioning successful";
            char* msg = new char[strlen(success) + 1];
            strcpy(msg, success);
            return {0, msg};

        } catch (const std::exception& e) {
            std::string what = e.what();
            char* msg = new char[what.length() + 1];
            strcpy(msg, what.c_str());
            return {1, msg};
        }
    }

    OSRM_Result osrm_run_customize(const char* base_path, int threads) {
        if (!base_path) {
            const char* err = "Path cannot be null";
            char* msg = new char[strlen(err) + 1];
            strcpy(msg, err);
            return {1, msg};
        }

        try {
            osrm::customizer::CustomizationConfig config;
            config.base_path = std::filesystem::path(base_path);
            config.UseDefaultOutputNames(config.base_path);
            
            // Set thread count: use provided value if > 0, otherwise use hardware concurrency
            if (threads > 0) {
                config.requested_num_threads = threads;
            } else {
                config.requested_num_threads = std::thread::hardware_concurrency();
            }

            osrm::customizer::Customizer customizer;
            int ret = customizer.Run(config);

            if (ret != 0) {
                const char* err = "Customize run returned non-zero code";
                char* msg = new char[strlen(err) + 1];
                strcpy(msg, err);
                return {ret, msg};
            }

            const char* success = "Customization successful";
            char* msg = new char[strlen(success) + 1];
            strcpy(msg, success);
            return {0, msg};

        } catch (const std::exception& e) {
            std::string what = e.what();
            char* msg = new char[what.length() + 1];
            strcpy(msg, what.c_str());
            return {1, msg};
        }
    }

    void osrm_free_string(char* s) {
        if (s) {
            delete[] s;
        }
    }
}
