#pragma once
#ifndef CATA_SRC_MAP_ROUTE_H
#define CATA_SRC_MAP_ROUTE_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "coordinates.h"
#include "optional.h"
#include "units.h"
#include "vehicle.h"

struct map_route_driving_data {
    // vehicle position (vehicle::pos)
    tripoint_abs_ms pos;
    // vehicle direction of travel
    units::angle target_heading;
    // cruise control target speed (vehicle::cruise_velocity)
    int target_speed;
    // index of the point in its parent route
    int cursor_pos;
};

struct map_route;

class map_route_manager
{
    public:
        map_route_manager();
        ~map_route_manager();
        map_route_manager( map_route_manager && );
        map_route_manager &operator=( map_route_manager && );


        void record_new_route();
        void add_recording_data( const vehicle &veh );
        void finish_recording_data( const vehicle &veh );

        std::vector<int> get_routes_starting_at( const vehicle &veh ) const;
        cata::optional<map_route_driving_data> get_driving_data( int route_id, int cursor_pos,
                const vehicle &veh ) const;

        static tripoint_abs_ms vehicle_pos( const vehicle &veh );
    private:
        int next_id = 1;
        std::unordered_map<int, std::unique_ptr<map_route>> all_map_routes;
        bool is_recording = false;
        std::unique_ptr<map_route> recording_route;
};

#endif // CATA_SRC_MAP_ROUTE_H
