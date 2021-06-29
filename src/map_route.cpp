#include "map_route.h"

#include <memory>

#include "coordinates.h"
#include "map.h"
#include "messages.h"
#include "optional.h"
#include "units.h"
#include "vehicle.h"

struct compact_point_data {
    int x, y;
    short z;
    short heading_degrees;
    short target_speed;
};

struct map_route {
    int id;
    std::vector<compact_point_data> points;
};

static map_route_driving_data expand( compact_point_data data, int cursor_pos )
{
    map_route_driving_data ret;
    ret.pos.raw() = tripoint( data.x, data.y, data.z );
    ret.target_heading = units::from_degrees( data.heading_degrees );
    ret.target_speed = data.target_speed;
    ret.cursor_pos = cursor_pos;
    return ret;
}

static compact_point_data get_data_from_vehicle( const vehicle &veh, bool stop = false )
{
    compact_point_data ret;
    tripoint_abs_ms pos = map_route_manager::vehicle_pos( veh );
    ret.x = pos.x();
    ret.y = pos.y();
    ret.z = pos.z();
    ret.heading_degrees = std::lround( units::to_degrees( veh.turn_dir ) );
    ret.target_speed = stop ? 0 : veh.cruise_velocity;
    return ret;
}

static bool same_place( const compact_point_data &a, const compact_point_data &b )
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}


map_route_manager::map_route_manager() = default;
map_route_manager::~map_route_manager() = default;
map_route_manager::map_route_manager( map_route_manager && ) = default;
map_route_manager &map_route_manager::operator=( map_route_manager && ) = default;

tripoint_abs_ms map_route_manager::vehicle_pos( const vehicle &veh )
{
    return tripoint_abs_ms( get_map().getabs( veh.global_pos3() ) );
}

void map_route_manager::record_new_route()
{
    is_recording = true;
    recording_route = std::make_unique<map_route>();
    add_msg("record_new_route");
}

void map_route_manager::add_recording_data( const vehicle &veh )
{
    if( !is_recording || !recording_route ) {
        return;
    }
    recording_route->points.emplace_back( get_data_from_vehicle( veh ) );
    const auto& p = recording_route->points.back();
    add_msg( "add_recording_data %d %d %d %d %d", p.x, p.y, p.z, p.heading_degrees, p.target_speed);
}

void map_route_manager::finish_recording_data( const vehicle &veh )
{
    if( !is_recording || !recording_route ) {
        return;
    }
    recording_route->points.emplace_back( get_data_from_vehicle( veh, true ) );

    if( recording_route->points.size() >= 2 ) {
        int new_id = next_id++;
        recording_route->id = new_id;
        all_map_routes[new_id].swap( recording_route );
        add_msg("finish_recording_data %d", new_id);
        for( const compact_point_data& p : all_map_routes[new_id]->points ) {
            add_msg("  %d %d %d %d %d", p.x, p.y, p.z, p.heading_degrees, p.target_speed);
        }
    }
    is_recording = false;
    recording_route = nullptr;
}

std::vector<int> map_route_manager::get_routes_starting_at( const vehicle &veh ) const
{
    std::vector<int> ret;
    compact_point_data veh_data = get_data_from_vehicle( veh );
    add_msg("get_routes_starting_at %d %d", veh_data.x, veh_data.y);
    for( const auto &iter : all_map_routes ) {
        const compact_point_data &route_start = iter.second->points[0];
        if( same_place( route_start, veh_data ) &&
            route_start.heading_degrees == veh_data.heading_degrees ) {
            ret.push_back( iter.first );
            add_msg( "  %d", ret.back());
        }
    }
    return ret;
}

cata::optional<map_route_driving_data> map_route_manager::get_driving_data( int route_id,
        int starting_cursor_pos, const vehicle &veh ) const
{
    auto iter = all_map_routes.find( route_id );
    if( iter == all_map_routes.end() ) {
        return cata::nullopt;
    }
    const map_route &route = *iter->second;
    const compact_point_data veh_data = get_data_from_vehicle( veh );
    for( int i = 0; i < 10; i++ ) {
        const int cursor_pos = starting_cursor_pos + i;
        if ( cursor_pos >= static_cast<int>( route.points.size() ) ) {
            break;
        }
        const compact_point_data &cursor_data = route.points[cursor_pos];
        if( same_place( veh_data, cursor_data ) ) {
            return expand( cursor_data, cursor_pos );
        }
    }
    return cata::nullopt;
}
