#define CATCH_CONFIG_MAIN

#include "../lib/catch.hpp"
#include "../src/tools.h"

TEST_CASE("CalculateRMSE calculate RMSE between estimations and ground_truth") {
    Tools tools;
    
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    
    SECTION("The estimations has size 0") {
        VectorXd g(4);
        g << 1.3, 1.3, 1.3, 1.3;
        ground_truth.push_back(g);
        
        VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
        VectorXd expected_rmse(4);
        expected_rmse << 0, 0, 0, 0;
        REQUIRE(rmse == expected_rmse);
    }
    
    SECTION("The estimations and group_truth differ in size") {
        VectorXd e(4);
        e << 1, 1, 1, 1;
        estimations.push_back(e);
        e << 1, 1, 1, 1;
        estimations.push_back(e);
        
        VectorXd g(4);
        g << 1.3, 1.3, 1.3, 1.3;
        ground_truth.push_back(g);
        
        VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
        VectorXd expected_rmse(4);
        expected_rmse << 0, 0, 0, 0;
        REQUIRE(rmse == expected_rmse);
    }
    
    SECTION("Good estimations and ground_truth") {
        //the input list of estimations
        VectorXd e(4);
        e << 1, 1, 1, 1;
        estimations.push_back(e);
        e << 1, 1, 1, 1;
        estimations.push_back(e);
        
        //the corresponding list of ground truth values
        VectorXd g(4);
        g << 1.3, 1.3, 1.3, 1.3;
        ground_truth.push_back(g);
        g << 0.7, 1.3, 1.3, 1.3;
        ground_truth.push_back(g);
        
        VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
        REQUIRE(rmse.size() == 4);
        CHECK(fabs(rmse(0)-0.3) < 0.0001);
        CHECK(fabs(rmse.sum()-1.2) < 0.0001);
    }
    

}
