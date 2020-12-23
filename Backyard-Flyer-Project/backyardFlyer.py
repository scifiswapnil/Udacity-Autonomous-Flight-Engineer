import time
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class Phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINTS = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.in_mission = True
        self.all_waypoints = [(0,0,3,0),(0,10,3,0),(10,10,3,0),(10,0,3,0)]
        self.waypoint = 4
        self.check_state = {}

        # initial state
        self.flight_phase = Phases.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY,
                               self.velocity_callback)
        self.register_callback(MsgID.STATE,
                               self.state_callback)

    def local_position_callback(self):
        altitude = -1.0 * self.local_position[2]
        if self.flight_phase == Phases.TAKEOFF or self.flight_phase == Phases.WAYPOINTS:
            if altitude > 0.95 * self.target_position[2] and self.target_position[0] > 0.95 * self.local_position[0] and self.target_position[1] > 0.95 * self.local_position[1]:
                self.waypoint_transition()
        elif self.flight_phase == Phases.LANDING:
            self.landing_transition()

    def velocity_callback(self):
        if self.flight_phase == Phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_phase == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == Phases.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_phase == Phases.DISARMING:
            if not self.armed:
                self.manual_transition()

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_phase = Phases.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_phase = Phases.TAKEOFF

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_phase = Phases.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_phase = Phases.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = Phases.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        super().start()
        self.stop_log()
    
    def waypoint_transition(self):
        self.flight_phase = Phases.WAYPOINTS
        altitude = -1.0 * self.local_position[2]
        if altitude > 0.95 * self.target_position[2] and abs(self.local_position[0] - self.target_position[0]) <0.3 and  abs(self.local_position[1] - self.target_position[1]) <0.3:
            self.waypoint -= 1
            print("waypoint",self.waypoint)
            if self.waypoint < 0:                
                self.flight_phase = Phases.LANDING
                print(self.local_position)
                return
            else:
                self.target_position[0] = self.all_waypoints[self.waypoint][0]
                self.target_position[1] = self.all_waypoints[self.waypoint][1]
            self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0)

if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(.5)
    drone.start()