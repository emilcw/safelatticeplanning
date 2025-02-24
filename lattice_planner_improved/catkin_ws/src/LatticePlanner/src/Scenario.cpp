#include "Scenario.h"

Scenario create_scenario(std::string scenario_name, std::string test_type, double parameter, int seed, bool predictable) {

  Scenario scenario(scenario_name);
  scenario.seed = seed;

  int maxValue = std::numeric_limits<int>::max()/2-1;

  // Default values
  scenario.start = create_point(0, -10, 1.5);
  scenario.goal = create_point(0, 8, 1.5);
  scenario.limits = create_AABB(-maxValue, maxValue, -maxValue, maxValue, -maxValue, maxValue);

  if(scenario.name == "empty") {
    /*    ---
     *   | ¤ |
     *    ---        */
    if(parameter < 0)
      parameter = 5;

    scenario.start = create_point(0, 0, 1.5);
    scenario.goal = create_point(2*parameter+2, 0, 1.5);
    scenario.limits = create_AABB(-parameter, parameter, -parameter, parameter, -parameter, parameter);
  }

  if(scenario.name == "dynamic") {
    /*          x
     *
     *              <-o
     *    o->
     *
     *          ¤           */
    StateSpaceModel * model1 = new ConstantVelocityModel(SearchState(-6,-4, 1.5, 0.3, 0, 0));
    scenario.obstacles.push_back(Obstacle( -6,-4, 2, 1,"constantVelocity", model1));

    StateSpaceModel * model2 = new ConstantVelocityModel(SearchState(6, 0, 1.5, -0.3, 0, 0));
    scenario.obstacles.push_back(Obstacle( 6, 0, 2, 1,"constantVelocity", model2));

    scenario.start = create_point(0, -10, 1.5);
    scenario.goal = create_point(0, 8, 1.5);
  }
  if(scenario.name == "dynamic2") {
      /*          x
       *
       *              <-o
       *    o->
       *
       *          ¤           */
      StateSpaceModel * model1;
      StateSpaceModel * model2;
      StateSpaceModel * model3;
      int N = 10;
      double distance = 4.6;
      int K = std::ceil(N*distance);
      for(double n = -K; n <= K; n += distance) {
        model1 = new ConstantVelocityModel2(SearchState(n,-4, 1.5, 0.0, 0, 0));
        scenario.obstacles.push_back(Obstacle( n,-4, 2, 1,"constantVelocity", model1));

        model2 = new ConstantVelocityModel2(SearchState(-n, 0, 1.5, -0.0, 0, 0));
        scenario.obstacles.push_back(Obstacle( -n, 0, 2, 1,"constantVelocity", model2));

        model3 = new ConstantVelocityModel2(SearchState(n,-8, 1.5, 0.0, 0, 0));
        scenario.obstacles.push_back(Obstacle( n,-8, 2, 1,"constantVelocity", model3));

      }

      scenario.start = create_point(0, -12, 1.5);
      scenario.goal = create_point(0, 8, 1.5);
      scenario.limits = create_AABB(-1000, 1000, -25, 10, 1.5, 1.5);
    }

  if(scenario.name == "wall2") {
    /*          x
     *
     *         ---
     *
     *          ¤           */
    scenario.limits = create_AABB(-2.5, 2.5, -10, 10, 0.5, 5);

    Obstacle wall1 = Obstacle( 0, 0, 2, -0.5,"wall");
    wall1.get_hitbox().add_AABB(3, 1, 1.5);
    wall1.display_solid = true;
    scenario.obstacles.push_back(wall1);
    Obstacle wall2 = Obstacle( -1, 0, 3.5, 0,"wall");
    wall2.get_hitbox().add_AABB(2, 1, 2.5);
    wall2.display_solid = true;
    scenario.obstacles.push_back(wall2);

    scenario.start = create_point(0, -6, 1.5);
    scenario.goal = create_point(0, 6, 1.5);
  }
  
  if(scenario.name == "wall") {
    /*          x
     *
     *         ---
     *
     *          ¤           */

    Obstacle wall1 = Obstacle( 0, 0, 1, 0,"wall");
    wall1.get_hitbox().add_AABB(2, 1, 5);
    scenario.obstacles.push_back(wall1);

    scenario.start = create_point(0, -10, 1.5);
    scenario.goal = create_point(0, 8, 1.5);
  }

  if(scenario.name == "wall3D") {
      /*          x
       *
       *         ---
       *
       *          ¤           */
    if(parameter < 0)
      parameter = 1;

    if(parameter > 0) {
      Obstacle wall1 = Obstacle( 0, 0, 1, 0,"wall");
      wall1.get_hitbox().add_AABB(parameter, 1, parameter);
      scenario.obstacles.push_back(wall1);
    }
    scenario.start = create_point(0, -10, 1.5);
    scenario.goal = create_point(0, 8, 1.5);
    scenario.limits = create_AABB(-maxValue, maxValue, -maxValue, maxValue, -maxValue, maxValue);
  }

  if(scenario.name == "moving_guard") {

    /*          x
     *
     * --------| |-------
     *        <-o
     *
     *          ¤           */
    Obstacle wall1 = Obstacle( -10.1, 0, 1, 1.5,"wall");
    wall1.get_hitbox().add_AABB(9, 1, 2);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( 10.1, 0, 1, 1.5,"wall");
    wall2.get_hitbox().add_AABB(9, 1, 2);
    scenario.obstacles.push_back(wall2);

    StateSpaceModel * model1 = new ConstantVelocityModel2(SearchState(1, -2, 2, -0.2, 0, 0));
    scenario.obstacles.push_back(Obstacle( 1, -2, 2, 1,"constantVelocity", model1));

    scenario.start = create_point(0, -4, 1.5);
    scenario.goal = create_point(0, 4, 1.5);
    scenario.limits = create_AABB(-22, 22, -10, 10, 1.5, 1.5);
  }

  if(scenario.name == "blind_corner") {

    Obstacle wall1 = Obstacle( -11.1, 0, 1, 1.5,"wall");
    wall1.get_hitbox().add_AABB(9, 1, 2);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( 11.1, 0, 1, 1.5,"wall");
    wall2.get_hitbox().add_AABB(9, 1, 2);
    scenario.obstacles.push_back(wall2);

    AABB human_limit = create_AABB(-10, 10, 2.5, 2.5, 1.5, 1.5);
    AABB uav_limit = create_AABB(-2, 2, -8, 8, 1.5, 1.5);
    scenario.obstacles.push_back(create_advanced_obstacle(-4,2.5,1.5, 1.0, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(4,2.5,1.5, 1.0, "human", human_limit, predictable));

    scenario.start = create_point(0, -4, 1.5);
    scenario.goal = create_point(0, 4, 1.5);
    scenario.limits = create_AABB(-10, 10, -10, 10, 1.5, 1.5);
  }

  if(scenario.name == "crossing_path") {
    /*
     *   ¤    <-o
     *               */
    StateSpaceModel * model1 = new ConstantVelocityModel2(SearchState(7, 0, 2, -0.5, 0, 0));
    scenario.obstacles.push_back(Obstacle( 7, 0, 2, 1,"constantVelocity", model1));

    scenario.start = create_point(0, 0, 1.5);
    scenario.goal = create_point(0, 0, 1.5);
    scenario.limits = create_AABB(-20, 20, -20, 20, 1.5, 1.5);
  }

  if(scenario.name == "corridor") {
    /* -----------------------
     * |¤            <-o   x |
     * --------   ------------
     *        |   |
     *        |   |
     *        |---|           */
    if(parameter < 0)
      parameter = 14;

    Obstacle wall1 = Obstacle( 0, 4.1, 1, 0,"wall1");   // top
    wall1.get_hitbox().add_AABB(7.1, 1, 3);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( -8.1, 2, 1, 0,"wall2");  // left
    wall2.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall2);

    Obstacle wall3 = Obstacle( 8.1, 2, 1, 0,"wall3");   // right
    wall3.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall3);

    Obstacle wall4 = Obstacle( -5.1, -0.1, 1, 0,"wall4");  // bottom left   ---| |===
    wall4.get_hitbox().add_AABB(2, 1, 3);
    scenario.obstacles.push_back(wall4);

    Obstacle wall5 = Obstacle( 5.1, -0.1, 1, 0,"wall5");  // bottom right   ===| |---
    wall5.get_hitbox().add_AABB(2, 1, 3);
    scenario.obstacles.push_back(wall5);

    Obstacle wall6 = Obstacle( -2.1, -parameter-0.1, 1, 0,"wall6");
    wall6.get_hitbox().add_AABB(1, parameter+1, 3);
    scenario.obstacles.push_back(wall6);

    Obstacle wall7 = Obstacle( 2.1, -parameter-0.1, 1, 0,"wall7");
    wall7.get_hitbox().add_AABB(1, parameter+1, 3);
    scenario.obstacles.push_back(wall7);

    Obstacle wall8 = Obstacle( 0, -2*parameter-0.1, 1, 0,"wall8");  // bottom
    wall8.get_hitbox().add_AABB(1.1, 1, 3);
    scenario.obstacles.push_back(wall8);

    StateSpaceModel * model2 = new ConstantVelocityModel2(SearchState(6, 2, 2, -0.45, 0, 0));
    //StateSpaceModel * model2 = new ConstantVelocityModel(SearchState(6, 2, 2, -0.45, 0, 0));
    scenario.obstacles.push_back(Obstacle( 6, 2, 2, 1,"constantVelocity", model2));

    scenario.start = create_point(-3, 2, 1.5);
    //start = create_point(0, -24, 2);
    scenario.goal = create_point(5, 2, 1.5);
    //goal = create_point(0, -10, 2);

    //lattice_planner_node->set_allowed_area(-8, 8, -7, 4, 1, 3);
    scenario.limits = create_AABB(-8, 8, -27, 4, 1.5, 1.5);
  }

  if(scenario.name == "corridor2") {
    /* -----------------------
     * |¤            <-o   x |
     * --------   ------------
     *        |   |
     *        |   |
     *        |---|           */
    // planning time := 0.8 is suitable

    int N = 3;  // Number of repetitions
    double width = 1.8*8.0;

    Obstacle wall2 = Obstacle( -8.1, 2, 1, 0,"wall2");
    wall2.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall2);

    Obstacle wall3 = Obstacle( 8.1+(N-1)*width, 2, 1, 0,"wall3");
    wall3.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall3);

    for(int n = 0; n < N; n++) {
      Obstacle wall1 = Obstacle( 0+n*width, 4.1, 1, 0,"wall1");
      wall1.get_hitbox().add_AABB(7.6, 1, 3);
      scenario.obstacles.push_back(wall1);

      Obstacle wall4 = Obstacle( -4.1+n*width, -0.1, 1, 0,"wall4");
      wall4.get_hitbox().add_AABB(3, 1, 3);
      scenario.obstacles.push_back(wall4);

      Obstacle wall5 = Obstacle( 4.1+n*width, -0.1, 1, 0,"wall5");
      wall5.get_hitbox().add_AABB(3, 1, 3);
      scenario.obstacles.push_back(wall5);

      Obstacle wall6 = Obstacle( -2.1+n*width, -4.1-10, 1, 0,"wall6");
      wall6.get_hitbox().add_AABB(1, 4+10, 3);
      scenario.obstacles.push_back(wall6);

      Obstacle wall7 = Obstacle( 2.1+n*width, -4.1-10, 1, 0,"wall7");
      wall7.get_hitbox().add_AABB(1, 4+10, 3);
      scenario.obstacles.push_back(wall7);

      Obstacle wall8 = Obstacle( 0+n*width, -27.1, 1, 0,"wall8");
      wall8.get_hitbox().add_AABB(3, 1, 3);
      scenario.obstacles.push_back(wall8);

      double k = 1.0;
      if(n == 1) continue;
      StateSpaceModel * model2 = new ConstantVelocityModel2(SearchState(6+n*width*k, 2, 2, -0.45, 0, 0));
      //StateSpaceModel * model2 = new ConstantVelocityModel(SearchState(6, 2, 2, -0.45, 0, 0));
      scenario.obstacles.push_back(Obstacle( 6+n*width*k, 2, 2, 1,"constantVelocity", model2));

    }

    scenario.start = create_point(-3, 2, 1.5);
    //start = create_point(0, -24, 2);
    scenario.goal = create_point(5+(N-1)*width, 2, 1.5);
    //goal = create_point(0, -10, 2);

    //lattice_planner_node->set_allowed_area(-8, 8, -7, 4, 1, 3);
    scenario.limits = create_AABB(-1000, 1000, -27, 4, 1.5, 1.5);
  }

  if(scenario.name == "corridor_fast") {
    /* -----------------------
     * |¤            <-o   x |
     * --------   ------------
     *        |   |
     *        |   |
     *        |---|           */

    Obstacle wall1 = Obstacle( 0, 4.1, 1, 0,"wall1");
    wall1.get_hitbox().add_AABB(8, 1, 3);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( -8.1, 2, 1, 0,"wall2");
    wall2.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall2);

    Obstacle wall3 = Obstacle( 8.1, 2, 1, 0,"wall3");
    wall3.get_hitbox().add_AABB(1, 3.1, 3);
    scenario.obstacles.push_back(wall3);

    Obstacle wall4 = Obstacle( -4.1, -0.1, 1, 0,"wall4");
    wall4.get_hitbox().add_AABB(3, 1, 3);
    scenario.obstacles.push_back(wall4);

    Obstacle wall5 = Obstacle( 4.1, -0.1, 1, 0,"wall5");
    wall5.get_hitbox().add_AABB(3, 1, 3);
    scenario.obstacles.push_back(wall5);

    Obstacle wall6 = Obstacle( -2.1, -4.1-10, 1, 0,"wall6");
    wall6.get_hitbox().add_AABB(1, 4+10, 3);
    scenario.obstacles.push_back(wall6);

    Obstacle wall7 = Obstacle( 2.1, -4.1-10, 1, 0,"wall7");
    wall7.get_hitbox().add_AABB(1, 4+10, 3);
    scenario.obstacles.push_back(wall7);

    Obstacle wall8 = Obstacle( 0, -27.1, 1, 0,"wall8");
    wall8.get_hitbox().add_AABB(3, 1, 3);
    scenario.obstacles.push_back(wall8);

    StateSpaceModel * model2 = new ConstantVelocityModel2(SearchState(6, 2, 2, -1.0, 0, 0));
    //StateSpaceModel * model2 = new ConstantVelocityModel(SearchState(6, 2, 2, -0.45, 0, 0));
    scenario.obstacles.push_back(Obstacle( 6, 2, 2, 1,"constantVelocity", model2));

    scenario.start = create_point(-3, 2, 1.5);
    //start = create_point(0, -24, 2);
    scenario.goal = create_point(5, 2, 1.5);
    //goal = create_point(0, -10, 2);

    //lattice_planner_node->set_allowed_area(-8, 8, -7, 4, 1, 3);
    scenario.limits = create_AABB(-8, 8, -27, 4, 1.5, 1.5);
  }

  if(scenario.name == "dead_end") {
    Obstacle wall1 = Obstacle( 5, -7, 2, 0,"wall1");
    wall1.get_hitbox().add_AABB(0.5, 10.5, 3);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( -1, 3, 2, 0,"wall2");
    wall2.get_hitbox().add_AABB(5.5, 0.5, 3);
    scenario.obstacles.push_back(wall2);

    scenario.start = create_point(0,0, 1.5);
    scenario.goal = create_point(7, 0, 1.5);
    scenario.limits = create_AABB(-1000, 1000, -1000, 1000, 1.5, 3.5);
  }

  if(scenario.name == "indoor1" || scenario.name == "indoor11" || scenario.name == "indoor2" || scenario.name == "indoor3" || scenario.name == "indoor_empty" || scenario.name == "indoor3_corridor") {
    /*         |--|            |--|
     * ---------  --------------  --------
     * |
     *
     * ..................................................
    */

    double s = 1.0;
    srand(seed);

    /* Top corridor */
    Obstacle wall1;
    Obstacle wall2;
    Obstacle wall3;
    Obstacle wall4;
    Obstacle wall5;
    Obstacle wall001;
    Obstacle wall002;

    if (scenario.name == "indoor11" || scenario.name == "indoor3" || scenario.name == "indoor3_corridor") {
      wall1 = Obstacle( -11.9*s, 11.1*s, 2, 0,"wall1");
      wall1.get_hitbox().add_AABB(2.5*s, 1.5, 3);
      scenario.obstacles.push_back(wall1);

      Obstacle wall2 = Obstacle( -3.9, 11.1*s, 2, 0,"wall2");
      wall2.get_hitbox().add_AABB(2.5*s, 1.5, 3);
      scenario.obstacles.push_back(wall2);

      wall001 = Obstacle( 3.9, 11.1*s, 2, 0,"wall001");
      wall001.get_hitbox().add_AABB(2.5*s, 1.5, 3);
      scenario.obstacles.push_back(wall001);

      wall3 = Obstacle( 11.9*s, 11.1*s, 2, 0,"wall3");
      wall3.get_hitbox().add_AABB(2.5, 1.5, 3);
      scenario.obstacles.push_back(wall3);

      wall002 = Obstacle( 0, 12.1*s, 2, 0,"wall002");
      wall002.get_hitbox().add_AABB(1.4, 0.5, 3);
      scenario.obstacles.push_back(wall002);

      wall4 = Obstacle( -7.9*s, 12.1*s, 2, 0,"wall4");
      wall4.get_hitbox().add_AABB(1.5*s, 0.5, 3);
      scenario.obstacles.push_back(wall4);

      wall5 = Obstacle( 7.9*s, 12.1*s, 2, 0,"wall5");
      wall5.get_hitbox().add_AABB(1.5, 0.5, 3);
      scenario.obstacles.push_back(wall5);
    }
    else {
      Obstacle wall1 = Obstacle( -10.2*s, 11.1*s, 2, 0,"wall1");
      wall1.get_hitbox().add_AABB(3.1*s, 1.5, 3);
      scenario.obstacles.push_back(wall1);

      Obstacle wall2 = Obstacle( 0, 11.1*s, 2, 0,"wall2");
      wall2.get_hitbox().add_AABB(4*s, 1.5, 3);
      scenario.obstacles.push_back(wall2);

      Obstacle wall3 = Obstacle( 10.2*s, 11.1*s, 2, 0,"wall3");
      wall3.get_hitbox().add_AABB(3.1, 1.5, 3);
      scenario.obstacles.push_back(wall3);

      Obstacle wall4 = Obstacle( -5.5*s, 12.1*s, 2, 0,"wall4");
      wall4.get_hitbox().add_AABB(1.5*s, 0.5, 3);
      scenario.obstacles.push_back(wall4);

      Obstacle wall5 = Obstacle( 5.5*s, 12.1*s, 2, 0,"wall5");
      wall5.get_hitbox().add_AABB(1.5, 0.5, 3);
      scenario.obstacles.push_back(wall5);
    }

    Obstacle wall6 = Obstacle( -13.9*s, 0, 2, 0,"wall6"); // Long left top
    wall6.get_hitbox().add_AABB(0.5, 10*s, 3);
    scenario.obstacles.push_back(wall6);
  /*
    Obstacle wall7 = Obstacle( -13.9*s, -5, 2, 0,"wall7");
    wall7.get_hitbox().add_AABB(0.5, 5*s, 3);
    scenario.obstacles.push_back(wall7);*/

    Obstacle wall8 = Obstacle( 0.4, 5.9*s, 2, 0,"wall8");
    wall8.get_hitbox().add_AABB(9*s, 0.5, 3);
    scenario.obstacles.push_back(wall8);

    /* Bottom Left rooms */
    Obstacle wall1_ = Obstacle( -10.4*s, -11.1*s, 2, 0,"wall1_");
    wall1_.get_hitbox().add_AABB(2.9*s, 0.5, 3);
    scenario.obstacles.push_back(wall1_);

    Obstacle wall2_ = Obstacle( 0, -11.1*s, 2, 0,"wall2_");
    wall2_.get_hitbox().add_AABB(3.9*s, 0.5, 3);
    scenario.obstacles.push_back(wall2_);

    Obstacle wall3_ = Obstacle( 10.4*s, -11.1*s, 2, 0,"wall3_");
    wall3_.get_hitbox().add_AABB(2.9, 0.5, 3);
    scenario.obstacles.push_back(wall3_);

    Obstacle wall4_ = Obstacle( 13.9*s, -18*s, 2, 0,"wall4_");
    wall4_.get_hitbox().add_AABB(0.5, 8*s, 3);
    scenario.obstacles.push_back(wall4_);

    Obstacle wall5_ = Obstacle( -13.9*s, -18.*s, 2, 0,"wall5_");
    wall5_.get_hitbox().add_AABB(0.5, 8*s, 3);
    scenario.obstacles.push_back(wall5_);

    Obstacle wall6_ = Obstacle( 0, -18.5*s, 2, 0,"wall6_");
    wall6_.get_hitbox().add_AABB(0.5, 7*s, 3);
    scenario.obstacles.push_back(wall6_);

    Obstacle wall7_ = Obstacle( 0, -25.6*s, 2, 0,"wall7_");
    wall7_.get_hitbox().add_AABB(13.5*s, 0.5, 3);
    scenario.obstacles.push_back(wall7_);

    int K = 2; // Old indoor scenario
    if (scenario.name == "indoor1" || scenario.name == "indoor11" || scenario.name == "indoor3" || scenario.name == "indoor3_corridor")
      K = 1; // New indoor scenario

    double ts = 27.8;
    for(int k = 1; k <= K; k++) {
      double t = ts*k;
      /* Bottom Right rooms */
      Obstacle wall1__ = Obstacle( (-10.4+t)*s, -11.1*s, 2, 0,"wall1__");
      wall1__.get_hitbox().add_AABB(2.9*s, 0.5, 3);
      scenario.obstacles.push_back(wall1__);

      Obstacle wall2__ = Obstacle( (0+t)*s, -11.1*s, 2, 0,"wall2__");
      wall2__.get_hitbox().add_AABB(3.9*s, 0.5, 3);
      scenario.obstacles.push_back(wall2__);

      Obstacle wall3__ = Obstacle( (10.4+t)*s, -11.1*s, 2, 0,"wall3__");
      wall3__.get_hitbox().add_AABB(2.9, 0.5, 3);
      scenario.obstacles.push_back(wall3__);

      Obstacle wall4__ = Obstacle( (13.9+t)*s, -18*s, 2, 0,"wall4__");
      wall4__.get_hitbox().add_AABB(0.5, 8*s, 3);
      scenario.obstacles.push_back(wall4__);
  /*
      Obstacle wall5__ = Obstacle( (-13.9+t)*s, -18.*s, 2, 0,"wall5__");
      wall5__.get_hitbox().add_AABB(0.5, 8*s, 3);
      scenario.obstacles.push_back(wall5__);
  */
      Obstacle wall6__ = Obstacle( (0+t)*s, -18.5*s, 2, 0,"wall6__");
      wall6__.get_hitbox().add_AABB(0.5, 7*s, 3);
      scenario.obstacles.push_back(wall6__);

      Obstacle wall7__ = Obstacle( (0+t)*s, -25.6*s, 2, 0,"wall7__");
      wall7__.get_hitbox().add_AABB(13.5*s, 0.5, 3);
      scenario.obstacles.push_back(wall7__);

      /*
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall1__);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall2__);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall3__);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall4__);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall6__);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall7__);*/
    }

    /* Right area */
    Obstacle wall9 = Obstacle( 9.9*s, 0, 2, 0,"wall9");
    wall9.get_hitbox().add_AABB(0.5, 6.4*s, 3);
    scenario.obstacles.push_back(wall9);

    Obstacle wall10 = Obstacle( 11.9, -5.9*s, 2, 0,"wall10");
    wall10.get_hitbox().add_AABB(1.5*s, 0.5, 3);
    scenario.obstacles.push_back(wall10);

    Obstacle wall11 = Obstacle( 13.9*s, -8.4*s, 2, 0,"wall11");
    wall11.get_hitbox().add_AABB(0.5, 2.4*s, 3);
    scenario.obstacles.push_back(wall11);

    Obstacle wall12 = Obstacle( (13.9+ts*K)*s, 0, 2, 0,"wall12"); // Long right top
    wall12.get_hitbox().add_AABB(0.5, 10*s, 3);
    scenario.obstacles.push_back(wall12);

    Obstacle wall13 = Obstacle( 0.5+(ts/2.0 + K*ts/2.0)*s, 10.1*s, 2, 0,"wall13"); // Right top
    wall13.get_hitbox().add_AABB((K*ts/2.0)*s, 0.5, 3);
    scenario.obstacles.push_back(wall13);

    // Default limits and start/goal
    scenario.limits = create_AABB(-1000, 1000, -1000, 1000, 1.5, 3.5);
    scenario.start = create_point(0,0, 1.5);
    scenario.goal = create_point(15, 0, 1.5);

    if (scenario.name == "indoor1" || scenario.name == "indoor11") {  // Randomized goals in medium-size scenario
      scenario.randomize_goal = true;
      if(parameter < 0)
        scenario.duration = 10*60*60; // X seconds max
      else
        scenario.duration = parameter;

      scenario.limits = create_AABB(-14, 41, -26, 11.5, 1.5, 3.5);  // Quad goals can be both high and low

      // NOTE: Hitboxes are really spheres, need to overestimate?
      AABB human_limit = create_AABB(-12, 38, -24, 8, 1.5, 1.5);
      AABB uav_limit = create_AABB(-12, 38, -24, 8, 2.5, 3.5);
      // NOTE: Real min/max x,y is: -12.5/40, -24/8, can't give random start pos because if it ends up outside?
//        SearchState goalState = draw_random_position(obstacle->limit);
//        goalState = round_search_state(goalState);

      scenario.obstacles.push_back(create_advanced_obstacle(-6,3, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(36,-5, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(30,-5, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(25,0, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(20,-20, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(-6,-15, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(32,-18, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(5,-10, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(35,5, 3.5, 0.4, "uav", uav_limit, predictable));
    } else if (scenario.name == "indoor2") {
      AABB human_limit = create_AABB(-15, 55, -35, 8, 1.5, 1.5);
      AABB uav_limit = create_AABB(-15, 55, -35, 8, 2.5, 3.5);
      scenario.obstacles.push_back(create_advanced_obstacle(-6,3,1.5, 0.45, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(50,-5,1.5, 0.45, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(40,-5,1.5, 0.45, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(45,0,1.5, 0.6, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(20,-20,1.5, 0.45, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(4,-20,1.5, 0.45, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(-6,-15, 3.0, 0.6, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.6, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(32,-18,3.5, 0.6, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(5,-10,3.5, 0.6, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(55,5,3.5, 0.6, "uav", uav_limit, predictable));
    } else if (scenario.name == "indoor3") {

      AABB human_limit = create_AABB(-12, 38, -24, 8, 1.5, 1.5);
      AABB uav_limit = create_AABB(-12, 38, -24, 8, 2.5, 3.5);
      // NOTE: Real min/max x,y is: -12.5/40, -24/8, can't give random start pos because if it ends up outside?
//        SearchState goalState = draw_random_position(obstacle->limit);
//        goalState = round_search_state(goalState);

      scenario.obstacles.push_back(create_advanced_obstacle(-6,3, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(36,-5, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(30,-5, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(25,0, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(20,-20, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(-6,-15, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(32,-18, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(5,-10, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(35,5, 3.5, 0.4, "uav", uav_limit, predictable));
    } else if (scenario.name == "indoor3_corridor") {
      // NOTE: Hitboxes are really spheres, need to overestimate?
      AABB human_limit = create_AABB(-12, 24, 0, 8, 1.5, 1.5);
      AABB uav_limit = create_AABB(-12, 24, 0, 8, 2.5, 3.5);
      // NOTE: Real min/max x,y is: -12.5/40, -24/8, can't give random start pos because if it ends up outside?
//        SearchState goalState = draw_random_position(obstacle->limit);
//        goalState = round_search_state(goalState);

      scenario.obstacles.push_back(create_advanced_obstacle(-6,3, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(24,1, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(20,2, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(21,1.5, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(22,1, 1.5, 0.3, "human", human_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(-6,2, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(20,2, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(5,3, 3.5, 0.4, "uav", uav_limit, predictable));
      scenario.obstacles.push_back(create_advanced_obstacle(21,5, 3.5, 0.4, "uav", uav_limit, predictable));

      scenario.limits = create_AABB(-14, 24, 0, 11.5, 1.5, 3.5);

      if(parameter < 0)
        scenario.duration = 1000000000; // X seconds max
      else
        scenario.duration = parameter;

      scenario.start = create_point(1.5, 8, 1.5);
      scenario.goal = create_point(1.5, 8, 1.5);
    }

    //StateSpaceModel * model1 = new ConstantVelocityModel2(SearchState(-6, 6, 2, 0, -0.45, 0));
    //scenario.obstacles.push_back(Obstacle( -6, 6, 2, 1,"constantVelocity", model1));

    //goal = create_point(25, -20, 1.5);
    //goal = create_point(1, 0, 1.5);


    /*
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall1);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall2);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall3);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall4);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall5);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall6);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall8);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall9);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall10);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall11);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall12);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall13);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall1_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall2_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall3_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall4_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall5_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall6_);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall7_);
    if(scenario.name == "indoor11") {
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall001);
      lattice_planner_node->object_lattice_planner_.add_obstacle(wall002);
    }
    */
  }
///////////// OLOAN TEST //////////////
if(scenario.name == "indoor_corner") {
  /*         |--|            |--|
   * ---------  --------------  --------
   * |
   *
   * ..................................................
  */

  double s = 1.0;
  srand(seed);

  /* Top corridor */
  Obstacle wall1;
  Obstacle wall2;
  Obstacle wall3;
  Obstacle wall4;
  Obstacle wall5;
  Obstacle wall001;
  Obstacle wall002;

  wall1 = Obstacle( -11.9*s, 11.1*s, 2, 0,"wall1");
  wall1.get_hitbox().add_AABB(2.5*s, 1.5, 3);
  scenario.obstacles.push_back(wall1);

  wall2 = Obstacle( -3.9, 11.1*s, 2, 0,"wall2");
  wall2.get_hitbox().add_AABB(2.5*s, 1.5, 3);
  scenario.obstacles.push_back(wall2);

  wall001 = Obstacle( 3.9, 11.1*s, 2, 0,"wall001");
  wall001.get_hitbox().add_AABB(2.5*s, 1.5, 3);
  scenario.obstacles.push_back(wall001);

  wall3 = Obstacle( 11.9*s, 11.1*s, 2, 0,"wall3");
  wall3.get_hitbox().add_AABB(2.5, 1.5, 3);
  scenario.obstacles.push_back(wall3);

  wall002 = Obstacle( 0, 12.1*s, 2, 0,"wall002");
  wall002.get_hitbox().add_AABB(1.4, 0.5, 3);
  scenario.obstacles.push_back(wall002);

  wall4 = Obstacle( -7.9*s, 12.1*s, 2, 0,"wall4");
  wall4.get_hitbox().add_AABB(1.5*s, 0.5, 3);
  scenario.obstacles.push_back(wall4);

  wall5 = Obstacle( 7.9*s, 12.1*s, 2, 0,"wall5");
  wall5.get_hitbox().add_AABB(1.5, 0.5, 3);
  scenario.obstacles.push_back(wall5);



  Obstacle wall6 = Obstacle( -13.9*s, 0, 2, 0,"wall6"); // Long left top
  wall6.get_hitbox().add_AABB(0.5, 10*s, 3);
  scenario.obstacles.push_back(wall6);
/*
  Obstacle wall7 = Obstacle( -13.9*s, -5, 2, 0,"wall7");
  wall7.get_hitbox().add_AABB(0.5, 5*s, 3);
  scenario.obstacles.push_back(wall7);*/
  // OLOAN NOTE: Obstacle parameters are x_pos and y_pos of obstacle center,
  // size (radius) in x,y is set by the hitbox
  Obstacle wall8 = Obstacle( 0.4+1, 5.9*s-2, 2, 0,"wall8");  // Corridor horizontal divider?
  wall8.get_hitbox().add_AABB(9*s, 0.5, 3);
  scenario.obstacles.push_back(wall8);

  /* Bottom Left rooms */
  Obstacle wall1_ = Obstacle( -10.4*s, -11.1*s, 2, 0,"wall1_");
  wall1_.get_hitbox().add_AABB(2.9*s, 0.5, 3);
  scenario.obstacles.push_back(wall1_);

  Obstacle wall2_ = Obstacle( 0, -11.1*s, 2, 0,"wall2_");
  wall2_.get_hitbox().add_AABB(3.9*s, 0.5, 3);
  scenario.obstacles.push_back(wall2_);

  Obstacle wall3_ = Obstacle( 10.4*s, -11.1*s, 2, 0,"wall3_");
  wall3_.get_hitbox().add_AABB(2.9, 0.5, 3);
  scenario.obstacles.push_back(wall3_);

  Obstacle wall4_ = Obstacle( 13.9*s, -18*s, 2, 0,"wall4_");
  wall4_.get_hitbox().add_AABB(0.5, 8*s, 3);
  scenario.obstacles.push_back(wall4_);

  Obstacle wall5_ = Obstacle( -13.9*s, -18.*s, 2, 0,"wall5_");
  wall5_.get_hitbox().add_AABB(0.5, 8*s, 3);
  scenario.obstacles.push_back(wall5_);

  Obstacle wall6_ = Obstacle( 0, -18.5*s, 2, 0,"wall6_");
  wall6_.get_hitbox().add_AABB(0.5, 7*s, 3);
  scenario.obstacles.push_back(wall6_);

  Obstacle wall7_ = Obstacle( 0, -25.6*s, 2, 0,"wall7_");
  wall7_.get_hitbox().add_AABB(13.5*s, 0.5, 3);
  scenario.obstacles.push_back(wall7_);

  int K = 1; // New indoor scenario

  double ts = 27.8;
  for(int k = 1; k <= K; k++) {
    double t = ts*k;
    /* Bottom Right rooms */
    Obstacle wall1__ = Obstacle( (-10.4+t)*s, -11.1*s, 2, 0,"wall1__");
    wall1__.get_hitbox().add_AABB(2.9*s, 0.5, 3);
    scenario.obstacles.push_back(wall1__);

    Obstacle wall2__ = Obstacle( (0+t)*s, -11.1*s, 2, 0,"wall2__");
    wall2__.get_hitbox().add_AABB(3.9*s, 0.5, 3);
    scenario.obstacles.push_back(wall2__);

    Obstacle wall3__ = Obstacle( (10.4+t)*s, -11.1*s, 2, 0,"wall3__");
    wall3__.get_hitbox().add_AABB(2.9, 0.5, 3);
    scenario.obstacles.push_back(wall3__);

    Obstacle wall4__ = Obstacle( (13.9+t)*s, -18*s, 2, 0,"wall4__");
    wall4__.get_hitbox().add_AABB(0.5, 8*s, 3);
    scenario.obstacles.push_back(wall4__);
/*
    Obstacle wall5__ = Obstacle( (-13.9+t)*s, -18.*s, 2, 0,"wall5__");
    wall5__.get_hitbox().add_AABB(0.5, 8*s, 3);
    scenario.obstacles.push_back(wall5__);
*/
    Obstacle wall6__ = Obstacle( (0+t)*s, -18.5*s, 2, 0,"wall6__");
    wall6__.get_hitbox().add_AABB(0.5, 7*s, 3);
    scenario.obstacles.push_back(wall6__);

    Obstacle wall7__ = Obstacle( (0+t)*s, -25.6*s, 2, 0,"wall7__");
    wall7__.get_hitbox().add_AABB(13.5*s, 0.5, 3);
    scenario.obstacles.push_back(wall7__);

    /*
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall1__);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall2__);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall3__);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall4__);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall6__);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall7__);*/
  }

  /* Right area */
  Obstacle wall9 = Obstacle( 9.9*s, 0-1.5, 2, 0,"wall9");  // Middle vertical divider
  wall9.get_hitbox().add_AABB(0.5, 6.4*s-1.5, 3);
  scenario.obstacles.push_back(wall9);

  Obstacle wall10 = Obstacle( 11.9, -5.9*s, 2, 0,"wall10");
  wall10.get_hitbox().add_AABB(1.5*s, 0.5, 3);
  scenario.obstacles.push_back(wall10);

  Obstacle wall11 = Obstacle( 13.9*s, -8.4*s, 2, 0,"wall11");
  wall11.get_hitbox().add_AABB(0.5, 2.4*s, 3);
  scenario.obstacles.push_back(wall11);

  Obstacle wall12 = Obstacle( (13.9+ts*K)*s, 0, 2, 0,"wall12"); // Long right top
  wall12.get_hitbox().add_AABB(0.5, 10*s, 3);
  scenario.obstacles.push_back(wall12);

  Obstacle wall13 = Obstacle( 0.5+(ts/2.0 + K*ts/2.0)*s, 10.1*s, 2, 0,"wall13"); // Right top
  wall13.get_hitbox().add_AABB((K*ts/2.0)*s, 0.5, 3);
  scenario.obstacles.push_back(wall13);

  // Default limits and start/goal
  scenario.limits = create_AABB(-13, 10, -10, 10, 1.5, 1.5);  //minX, maxX,...
  scenario.start = create_point(0,0, 1.5);
  scenario.goal = create_point(5, 0, 1.5);

  AABB human_limit = create_AABB(-13, 10, -10, 8, 1.5, 1.5);
  AABB uav_limit = create_AABB(-12, 38, -24, 8, 2.5, 3.5);

  // AABB human_limit = create_AABB(-12, 38, -24, 8, 1.5, 1.5);
  // AABB uav_limit = create_AABB(-12, 38, -24, 8, 2.5, 3.5);
  // NOTE: Real min/max x,y is: -12.5/40, -24/8, can't give random start pos because if it ends up outside?
//        SearchState goalState = draw_random_position(obstacle->limit);
//        goalState = round_search_state(goalState);

  scenario.obstacles.push_back(create_advanced_obstacle(-6,3, 1.5, 0.3, "human", human_limit, predictable));
  scenario.obstacles.push_back(create_advanced_obstacle(36,-5, 1.5, 0.3, "human", human_limit, predictable));
  scenario.obstacles.push_back(create_advanced_obstacle(30,-5, 1.5, 0.3, "human", human_limit, predictable));
  scenario.obstacles.push_back(create_advanced_obstacle(25,0, 1.5, 0.3, "human", human_limit, predictable));
  scenario.obstacles.push_back(create_advanced_obstacle(20,-20, 1.5, 0.3, "human", human_limit, predictable));
  // scenario.obstacles.push_back(create_advanced_obstacle(-6,-15, 3.0, 0.4, "uav", uav_limit, predictable));
  // scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.4, "uav", uav_limit, predictable));
  // scenario.obstacles.push_back(create_advanced_obstacle(32,-18, 3.5, 0.4, "uav", uav_limit, predictable));
  // scenario.obstacles.push_back(create_advanced_obstacle(5,-10, 3.5, 0.4, "uav", uav_limit, predictable));
  // scenario.obstacles.push_back(create_advanced_obstacle(35,5, 3.5, 0.4, "uav", uav_limit, predictable));

    // scenario.limits = create_AABB(-14, 24, 0, 11.5, 1.5, 3.5);
    //
    // if(parameter < 0)
    //   scenario.duration = 1000000000; // X seconds max
    // else
    //   scenario.duration = parameter;
    //
    // scenario.start = create_point(1.5, 8, 1.5);
    // scenario.goal = create_point(1.5, 8, 1.5);

}
///////////////////////////







  if(scenario.name == "indoor_corridor") {
    /*         |--|            |--|
     * ---------  --------------  --------
     * o->              ¤              <-o
     * ...................................
    */

    double s = 1.0;
    srand(seed);

    /* Top corridor */

    Obstacle wall1 = Obstacle( -10.2*s, 11.1*s, 2, 0,"wall1");
    wall1.get_hitbox().add_AABB(3.1*s, 1.5, 3);
    scenario.obstacles.push_back(wall1);

    Obstacle wall2 = Obstacle( 0, 11.1*s, 2, 0,"wall2");
    wall2.get_hitbox().add_AABB(4*s, 1.5, 3);
    scenario.obstacles.push_back(wall2);

    Obstacle wall3 = Obstacle( 10.2*s, 11.1*s, 2, 0,"wall3");
    wall3.get_hitbox().add_AABB(3.1, 1.5, 3);
    scenario.obstacles.push_back(wall3);

    Obstacle wall4 = Obstacle( -5.5*s, 12.1*s, 2, 0,"wall4");
    wall4.get_hitbox().add_AABB(1.5*s, 0.5, 3);
    scenario.obstacles.push_back(wall4);

    Obstacle wall5 = Obstacle( 5.5*s, 12.1*s, 2, 0,"wall5");
    wall5.get_hitbox().add_AABB(1.5, 0.5, 3);
    scenario.obstacles.push_back(wall5);

    Obstacle wall6 = Obstacle( 0, 5.9*s, 2, 0,"wall6");
    wall6.get_hitbox().add_AABB(15*s, 0.5, 3);
    scenario.obstacles.push_back(wall6);


    AABB human_limit = create_AABB(-15, 15, 5, 12, 1.5, 1.5);
    AABB uav_limit = create_AABB(-15, 15, 5, 12, 2.5, 3.5);

    scenario.obstacles.push_back(create_advanced_obstacle(-6,3,1.5, 0.45, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(50,-5,1.5, 0.45, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(40,-5,1.5, 0.45, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(45,0,1.5, 1.0, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(20,-20,1.5, 0.45, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(4,-20,1.5, 0.45, "human", human_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(-6,-15, 3.0, 0.6, "uav", uav_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(7.9,3.9, 3.0, 0.6, "uav", uav_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(32,-18,3.5, 1.0, "uav", uav_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(5,-10,3.5, 1.0, "uav", uav_limit, predictable));
    scenario.obstacles.push_back(create_advanced_obstacle(55,5,3.5, 1.0, "uav", uav_limit, predictable));

    //StateSpaceModel * model1 = new ConstantVelocityModel2(SearchState(-6, 6, 2, 0, -0.45, 0));
    //scenario.obstacles.push_back(Obstacle( -6, 6, 2, 1,"constantVelocity", model1));

    scenario.start = create_point(0,0, 1.5);
    scenario.goal = create_point(15, 0, 1.5);
    //goal = create_point(25, -20, 1.5);
    //goal = create_point(1, 0, 1.5);
    scenario.limits = create_AABB(-1000, 1000, -1000, 1000, 1.5, 3.5);
    /*
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall1);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall2);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall3);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall4);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall5);
    lattice_planner_node->object_lattice_planner_.add_obstacle(wall6);
    */
  }

  if(scenario.name == "blocking_wall") {
    Obstacle wall1 = Obstacle( 0, 0, 1, 0,"wall");
    wall1.get_hitbox().add_AABB(10, 1, 5);
    scenario.obstacles.push_back(wall1);
    scenario.limits = create_AABB(-10, 10, -10, 10, 1.5, 1.5);

    scenario.start = create_point(0, -5, 1.5);
    scenario.goal = create_point(0, 5, 1.5);
  }

  return scenario;
}
