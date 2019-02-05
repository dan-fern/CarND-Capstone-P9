#!/usr/bin/env python

import rospy as rp
import numpy as np
import math as math
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

# Number of waypoints we will publish.
LOOKAHEAD_WPS = 150
MAX_DECEL = 0.5


class MotionState( ):
    Go, Stop = range( 2 )


class WaypointUpdater( object ):

    def __init__( self ):

        rp.init_node( 'waypoint_updater' )

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rp.Subscriber( '/current_pose', PoseStamped, self.pose_cb )
        rp.Subscriber( '/base_waypoints', Lane, self.waypoints_cb )
        rp.Subscriber( '/traffic_waypoint', Int32, self.traffic_cb )
        rp.Subscriber( '/current_velocity', TwistStamped, self.velocity_cb )

        self.final_waypoints_pub = rp.Publisher(
                'final_waypoints',
                Lane,
                queue_size=1 )

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None

        self.waypoints_2d = None
        self.waypoint_tree = None
        self.nearest_light = None
        self.vehicle_velocity = None # in m/s

        self.motion_state = MotionState.Go
        self.deceleration_rate = None
        self.acceleration_rate = 0.75 # m/s

        self.previous_velocity = None

        self.loop( )


    def loop( self ):
        rate = rp.Rate( 10 )

        while not rp.is_shutdown( ):

            if self.pose and self.base_lane and self.waypoint_tree:
                # get closest waypoint
                #closest_waypoint_index = self.get_closest_waypoint_id( )
                self.publish_waypoints( )

            self.previous_velocity = self.vehicle_velocity

            rate.sleep( )


    def publish_waypoints( self ):
        self.final_waypoints_pub.publish( self.generate_lane( ) )


    def generate_lane( self ):
        lane = Lane( )

        closest_idx = self.get_closest_waypoint_id( )
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane[ closest_idx:farthest_idx ]

        if self.nearest_light != None and \
                self.nearest_light <= farthest_idx and \
                self.nearest_light >= closest_idx:
            self.motion_state = MotionState.Stop
            base_waypoints = self.decelerate( base_waypoints, closest_idx )
        elif self.motion_state == MotionState.Stop:
            self.motion_state = MotionState.Go
            self.deceleration_rate = None

        if self.motion_state == MotionState.Go:
            if abs( self.vehicle_velocity - self.get_waypoint_velocity( \
                    base_waypoints[ 0 ] ) ) > 1.0:
                if self.previous_velocity == None:
                    start_vel = self.vehicle_velocity
                else:
                    start_vel = max(
                            self.previous_velocity + 0.2,
                            self.vehicle_velocity )
                base_waypoints = self.accelerate( base_waypoints, start_vel )
            else:
                self.acceleration_start_velocity = None

        lane.waypoints = base_waypoints

        return lane


    def accelerate( self, waypoints, start_velocity ):
        new_waypoints = [ ]

        for i, wp in enumerate( waypoints ):

            p = Waypoint( )
            p.pose = wp.pose

            distance = self.distance( waypoints, 0, i )
            target_vel = start_velocity + distance * self.acceleration_rate

            if target_vel < 0.5:
                target_vel = 0.5

            p.twist.twist.linear.x = min(
                    target_vel,
                    self.get_waypoint_velocity( wp ) )
            new_waypoints.append( p )

        return new_waypoints


    def decelerate( self, waypoints, start_idx ):
        new_waypoints = [ ]
        speed = self.vehicle_velocity
        # two waypoints back from line so front of car stops earlier
        stop_idx = self.nearest_light - start_idx - 2

        for i, wp in enumerate( waypoints ):

            p = Waypoint( )
            p.pose = wp.pose

            dist = self.distance( waypoints, i, stop_idx )
            if i >= stop_idx:
                target_vel = 0
            elif dist < 15:
                if self.deceleration_rate == None:
                    self.deceleration_rate = self.vehicle_velocity / dist
                target_vel = self.deceleration_rate * dist
                if target_vel <= 1.0:
                    target_vel = 0.0
                target_vel = min( target_vel, self.get_waypoint_velocity( wp ) )
            else:
                target_vel = self.get_waypoint_velocity( wp )

            p.twist.twist.linear.x = target_vel
            new_waypoints.append( p )

        return new_waypoints


    def get_closest_waypoint_id( self ):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query( [x, y], 1 )[1]

        # Check if closest waypoint is ahead or behind the vehicle
        closest_wp = np.array( self.waypoints_2d[ closest_idx ] )
        previous_wp = np.array( self.waypoints_2d[ closest_idx - 1 ] )

        # Equation for hyperplane through closest_coords
        waypoint_vector = closest_wp - previous_wp
        position_vector = np.array( [x, y] ) - closest_wp

        val = np.dot( waypoint_vector, position_vector )

        if val > 0:
            closest_idx = ( closest_idx + 1 ) % len( self.waypoints_2d )

        return closest_idx


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg


    def waypoints_cb( self, waypoints ):
        # TODO: Implement
        self.base_lane = waypoints.waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [ [ waypoint.pose.pose.position.x,
                    waypoint.pose.pose.position.y ]
                    for waypoint in waypoints.waypoints ]

            self.waypoint_tree = KDTree( self.waypoints_2d )


    def traffic_cb( self, msg ):
        # TODO: Callback for /traffic_waypoint message. Implement
        if( msg.data == -1 ):
            self.nearest_light = None
        else:
            self.nearest_light = msg.data


    def velocity_cb( self, velocity ):
        self.vehicle_velocity = velocity.twist.linear.x


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity( self, waypoint ):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity( self, waypoints, waypoint, velocity ):
        waypoints[ waypoint ].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rp.ROSInterruptException:
        rp.logerr('Could not start waypoint updater node.')
