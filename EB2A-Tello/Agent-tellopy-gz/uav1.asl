//////////////// Initial beliefs
status("None").
world_area(250, 250, 0, 0).
num_of_uavs(1).
camera_range(5).
std_altitude(20.0).
std_heading(0.0).
land_point(-102.0, -111.0).
land_radius(10.0).
diff(200.0).
thrs(200).
my_number(1). 
my_frame_id("uav1/gps_origin").

location(wp1,  0.0,  0.9, 0.48).
location(wp2,  0.0,  0.0, 0.48).
location(wp3,  0.6,  0.9, 0.48).
//location(wp4,  0.6,  0.9, 0.48).

firstStation(wp1).   

// Determine the last station to be visited
lastStation(wp3).

// Determine the next location to work (second parameter) based on the current one (first parameter)
next_location(wp1,wp2).
next_location(wp2,wp3).
//next_location(wp3,wp4).

//pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW))))
//////////////// Rules
//current_position(CX, CY, CW) :- pos_x(CX) & pos_y(CY) & pos_W(CW).
current_position(CX, CY, RW) :- pos_x(CX) & pos_y(CY) & pos_w(RW).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & drone1_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav7_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav8_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav9_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav10_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav11_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
//current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & uav12_odometry_gps_local_odom(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
near(X, Y, W) :- current_position(CX, CY, RW)
              & diff(D)
              & math.abs(CX - X) <= D
              & math.abs(CY - Y) <= D
              & math.abs(RW - W) <= D.

lightDetect(N) :- detectred(N)
               & diff(D)
               & math.abs(N) >= D.
my_number_string(S) :- my_number(N)
                       & .term2string(N, S).

+detect_fire_uav1(N) : my_number(N) <- !detected_fire(N).
//+detect_fire_uav2(N) : my_number(N) <- !detected_fire(N).
//+detect_fire_uav3(N) : my_number(N) <- !detected_fire(N).
//+detect_fire_uav4(N) : my_number(N) <- !detected_fire(N).
//+detect_fire_uav5(N) : my_number(N) <- !detected_fire(N).
//+detect_fire_uav6(N) : my_number(N) <- !detected_fire(N).

+temp(T1) : temp_limit(T2) & T1 >=T2  <- !temp_alarm(T1).

+wind(W1) : wind_limit(W2) & W1 >=W2  <- !wind_alarm(W1).

+goland(N) : my_number(N) <- !start_land(N).

//+detectred(N) : my_number(N) <- !reactRed(N).
+detectred(N) : thrs(CT) & N>=CT  <- !reactRed(N).

//+detectblue(N) : my_number(N) <- !reactBlue(N).
+detectblue(N) : thrs(CT) & N>=CT  <- !reactBlue(N).

+eland(N) : my_number(N) <- !elanding(N).
//+failure_uav1(N) : my_number(N) <- !detected_failure(N).

//+failure_uav1(N)<- !detected_failure.
+block(N) <- +failure.
+unblock(N) <- +unblocked.

//////////////// Start
//+!start
    //<- .wait(5000);
       //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1", "land",[]);
      //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[0.0, 0.0, 0.0]);
      //.print("Started!");
      //!calculate_trajectory;//trajectory//!calculate_area;//!calculate_waypoints(1, []);// pode ser unido com os outros
      //!hover.
      //!follow_trajectory(0).
      
!start.
+!start : firstStation(ST) <- .print("Manager agent started"); !work(ST).

      //!follow_trajectory(0).      

//+goto(X,Y)
//   :  not reachedGoal
//   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","go_to","" X ";" Y "");
//      !goto(X,Y).

+!reactRed(N)
   :  not reactblue
      & not reactred
      & afterhover
   <- +reactred;
      +afterred;
      .print("reactRed");
      .suspend(reactBlue(N));
      .suspend(hover);
      !left;
      .wait(7500);
      -reactred;
      !land.


+!reactBlue(N)
   :  afterhover
      & not reactred
      & not reactblue
   <- +reactblue;
      .suspend(reactRed(N));
      .print("reactBlue");
      .suspend(hover);
      !back;
      .wait(7500);
      !front;
      .wait(7500);
      -reactblue;
      !hover.
+!hover
   :  not reactblue
      & not reactred
   <- -+status("hover");
      .print("hover");
      +afterhover;
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","keepalive;0");
      .wait(1000);
      !hover.

+!work(Location) : location(Location, X, Y, W) 
	<- .wait(2000);
		-working(_);
		+working(Location);
		.print("Now moving to: ", X, ",",Y, ",",W);
		!goto(X, Y, W).   

+!elanding(N)
   :  my_number(N)
   <- +elanding;
      .suspend(hover);
      .suspend(reactRed(N));
      .suspend(reactBlue(N));
      !land.

//////////////// Check Near
+!goto(X, Y, W)
   :  near(X, Y, W)
      & working(Location) 
      & next_location(Location,Next)
   <- .print("Arrived at ", X, ",",Y);
      .wait(1000);
      !work(Next).



+!goto(X, Y, W)
   :  near(X, Y, W)
      & working(Location) 
      & next_location(Location,Next)
      & lastStation(LastLoc) 
      & Location == LastLoc
   <- .print("Finished Trajectory");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","land",N).

+!goto(X, Y, W)
   :  .term2string([X, Y, W], S)
      & current_position(CX, CY, RW)
      //& pos_x(CX)
      //& pos_y(CY)
      //& pos_w(CW)
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto",S);
      //.print("At pos: ", CX, ",",CY, ",",RW);
      //.print("Near next wp: ", near(X, Y, W));
      .wait(100);
      !goto(X, Y, W).

+failure
   <- .print("Suspending Trajectory!");
      -unblocked.
      
+unblocked
   <- .print("Resuming Trajectory!");
      .wait(2000);
      -failure.
+!start_land(N)
   <- .print("Resuming Trajectory!");  
      .print("land");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","land;0").

//embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","update_time",Msg).
//////////////// Calculating land position
+!takeoff
   <- -+status("taking off");
      .print("taking off");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","takeoff;0");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 

+!down
   <- -+status("down");
      .print("down");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","down;20");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 

+!up
   <- -+status("up");
      .print("up");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","up;20");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 
+!spin
   <- -+status("spin");
      .print("spin");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","telloAction","rc 0.01 0 0 0");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 

+!left
   <- -+status("left");
      .print("left");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","move_left;100");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 

+!right
   <- -+status("right");
      .print("right");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd_vel",[0,0.1,0,0]);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 
+!back
   :  my_number(N)
   <- -+status("back");
      .print("back");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","move_back;180");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset").  
+!front
   :  my_number(N)
   <- -+status("front");
      .print("front");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","move_forward;180");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset").  
      
+!back2
   <- -+status("back");
      .print("back");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd_vel",[-0.1,0,0,0]).   

+!rotate180
   :  my_number(N)
   <- -+status("rotate180");
      .print("rotate180");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","rotate_cw;180");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset"). 

                
+!land
   <- -+status("land");
      .print("land");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","land;0");
      .wait(100);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","cmd","reset").   




   
//Reaction -- TrajectMixStd
//+!detected_failure
//   <- critReac0.
 
//Reaction -- TrajectMixCrit
// Nao precisa de nada, faz pelo RosEnv
   
//Reaction -- TrajectEMASStd
//+!detected_failure
//   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","adf",N).
      
//Reaction -- TrajectEMASCrit
+cb0 
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","adf",N).      



//Reaction -- TrajectEMASStd
      
+!temp_alarm(T1)
   <- .print("Temp alarm: ",T1).
   
+!wind_alarm(W1)
   <- .print("Wind alarm: ",W1).

+!calculate_trajectory
   :  my_number(N)
      & landing_x (LX)
      & landing_y (LY)
      & land_radius(R)
      & num_of_uavs(NumOfUavs)
      & world_area(H, W, CX, CY)
   <- .print("Calculating landing position");
      -+status("calculating_land_position");
      LndNumOfColumns = NumOfUavs/2;
      LndRectangleHeight = R/2;
      LndRectangleWidth = R/LndNumOfColumns;
      My_landing_x = LX - R/2 + LndRectangleWidth/2 + ((N-1) mod LndNumOfColumns)*LndRectangleWidth;
      My_landing_y = LY - R/2 + LndRectangleHeight/2 + (math.floor((N-1)/LndNumOfColumns))*LndRectangleHeight;
      +my_landing_position(My_landing_x, My_landing_y);
      //////////////// Calculating area
      .print("Calculating area");
      +status("calculating_area");
      AreaNumOfColumns = NumOfUavs/2;
      AreaRectangleHeight = H/2;
      AreaRectangleWidth = W/AreaNumOfColumns;
      X1 = CX - W/2 + ((N-1) mod AreaNumOfColumns)*AreaRectangleWidth;
      X2 = CX - W/2 + ((N-1) mod AreaNumOfColumns + 1)*AreaRectangleWidth;
      Y1 = CY - H/2 + (math.floor((N-1)/AreaNumOfColumns))*AreaRectangleHeight;
      Y2 = CY - H/2 + (math.floor((N-1)/AreaNumOfColumns) + 1)*AreaRectangleHeight;
      +my_area(X1, X2, Y1, Y2);
      //////////////// Calculating waypoints
      !calculate_waypoints(1, []).



//////////////// Calculating waypoints
+!calculate_waypoints(C, OldWayList)
    :   camera_range(CR)
        & my_area(X1, X2, Y1, Y2)
        & X2 - (C+2)*CR/2 >= X1
        & std_altitude(Z)
    <-  .print("Calculating waypoints");
        -+status("calculating_waypoints");
        Waypoints = [
                        [X1 + C*CR/2, Y1 + CR/2, Z]
                        , [X1 + C*CR/2, Y2 - CR/2, Z]
                        , [X1 + (C+2)*CR/2, Y2 - CR/2, Z]
                        , [X1 + (C+2)*CR/2, Y1 + CR/2, Z]
                    ];
        .concat(OldWayList, Waypoints, NewWayList);
        !calculate_waypoints(C+4, NewWayList).

+!calculate_waypoints(_, WayList)
    <-  .print("Finished calculating waypoints");
        +waypoints_list(WayList);
        +waypoints_list_len(.length(WayList));
        .print("Waypoints list: ", WayList).


//////////////// Follow trajectory
+!follow_trajectory(CW)
   :  waypoints_list_len(CW)
      & my_number(N)
   <- .broadcast(tell, finished_trajectory(N));
      +finished_trajectory(N);
      -+status("finished_trajectory");
      .print("finished_trajectory");
      !wait_for_others.

+!follow_trajectory(CW)
   :  waypoints_list(WL)
      & waypoints_list_len(Len)
      & CW < Len
   <- -+status("following_trajectory");
      .print("following_trajectory");
      .nth(CW, WL, [X, Y, Z]);
      !check_near(X, Y, Z, "waypoint");
      !follow_trajectory(CW+1).


//////////////// Waiting
+!wait_for_others
   :  my_number(N)
      & my_landing_position(LAX, LAY)
      & .count(finished_trajectory(_), C)
      & num_of_uavs(C)
   <- .print("All finished, going to land position");
      !goto_landing_position(LAX, LAY).

+!wait_for_others
   <- -+status("waiting");
      .print("Still waiting");
      .wait(1000);
      !wait_for_others.


//////////////// Landing
+!goto_landing_position(X, Y)
   : std_altitude(Z)
   <- -+status("going_to_land_position");
      !check_near(X, Y, Z, "land position");
      !land.

+!land
   :  my_number_string(N)
   <- .print("Landing");
      -+status("landing");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1", "land", [N]).


//////////////// Fire Strategy
+!detected_fire(N)
   :  my_number(N)
      & current_position(CX, CY, CZ)
      & .intend(follow_trajectory(CW))
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(follow_trajectory(CW));
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending trajectory.");
      .broadcast(tell, found_fire(N, CX, CY));
      !combat_fireR(CW).
      //.wait(10000);   
      //+fire_extinguished;
      //.resume(follow_trajectory(CW));
      //.print("Fire extinguished. Resuming trajectory").


+found_fire(N, X, Y)
   : not my_number(N)
      & .intend(follow_trajectory(CW))
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(follow_trajectory(CW));
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending trajectory.");
      !goto_fire_position(X+N, Y, 15);
	  //acao de combate ao fogo/ na simulacao muda a cor do VANT/ no caso da implementacao real tem que ter uma funcao
      !combat_fireR(CW).
      //.wait(10000);
      //+fire_extinguished;
      //.resume(follow_trajectory(CW));
      //.print("Fire extinguished. Resuming trajectory").

+found_fire(N, X, Y)
   : not my_number(N)
      & .intend(wait_for_others)
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(wait_for_others);
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending waiting.");
      !goto_fire_position(X+N, Y, 15);
	  //acao de combate ao fogo/ na simulacao muda a cor do VANT/ no caso da implementacao real tem que ter uma funcao
      !combat_fire.
      

+!goto_fire_position(X, Y, Z)
   <- !check_near(X, Y, Z, "fire position").

//////////////// Combat Fire
      //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, CZ]);

+!combat_fire
   : current_position(CX, CY, CZ)
   <- .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, 8.0]);
      +fire_extinguished;
      .resume(wait_for_others);
      .print("Fire extinguished. Resuming waiting").
      
      
+!combat_fireR(CW)
   : current_position(CX, CY, CZ)
   <- .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, 8.0]);
      +fire_extinguished;
      .resume(follow_trajectory(CW));
      .print("Fire extinguished. Resuming trajectory").      
      

//////////////// Check Near
+!check_near(X, Y, Z, S)
   :  near(X, Y)
   <- .print("Arrived at ", S).

+!check_near(X, Y, Z, S)
   :  my_number_string(N)
      & std_heading(Heading)//+failure_uav1(N) Include failure state blocking goto plan
      & not failure
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto", [ X, Y, Z, Heading]);
      .wait(100);
      !check_near(X, Y, Z, S).


    
+!check_near(X, Y, Z, S)
   :  my_number_string(N)
      & std_heading(Heading)//+failure_uav1(N) Include failure state blocking goto plan
      & failure
  <- .wait(500);
     !check_near(X, Y, Z, S).
      
//////////////// Handling plan failure
+!detected_failure(_).
+!detected_fire(_).
+!found_fire(_, _, _).
+!reactRed(_).
+!reactBlue(_).
+!hover(_).
