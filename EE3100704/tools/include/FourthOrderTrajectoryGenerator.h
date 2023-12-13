//
// Created by percy on 23. 12. 8.
//

#ifndef EE3100704_PROJECTS_FOURTHORDERTRAJECTORYGENERATOR_H
#define EE3100704_PROJECTS_FOURTHORDERTRAJECTORYGENERATOR_H

#include <matplotlibcpp.h>
#include <iostream>
#include <cmath>

class FourthOrderTrajectoryGenerator {
public:
    // Constructor
    FourthOrderTrajectoryGenerator(double initial_time, double final_time,
                                   double initial_force, double final_force,
                                   double initial_velocity, double final_velocity,
                                   double initial_acceleration, double final_acceleration)
            : t0(initial_time), tf(final_time),
              q0(initial_force), qf(final_force),
              v0(initial_velocity), vf(final_velocity),
              a0(initial_acceleration), af(final_acceleration) {}

    // Generate the trajectory at time t
    double generateTrajectory(double t) {
        if (t < t0 || t > tf) {
            // Time is outside the trajectory interval
            return 0.0;
        }

        // Calculate normalized time within the trajectory interval
        double tau = (t - t0) / (tf - t0);

        // Calculate coefficients for the 4th order polynomial
        double a = 6 / std::pow(tf - t0, 4) * (qf - q0 - (v0 + vf) * (tf - t0) - (a0 - af) * std::pow(tf - t0, 2));
        double b = -15 / std::pow(tf - t0, 4) * (qf - q0 - (v0 + vf) * (tf - t0) - (a0 - af) * std::pow(tf - t0, 2));
        double c = 10 / std::pow(tf - t0, 4) * (qf - q0 - (v0 + vf) * (tf - t0) - (a0 - af) * std::pow(tf - t0, 2));
        double d = 0;
        double e = q0;

        // Evaluate the 4th order polynomial at time tau
        double Force = a * std::pow(tau, 4) + b * std::pow(tau, 3) + c * std::pow(tau, 2) + d * tau + e;

        return Force;
    }

private:
    double t0;  // Initial time
    double tf;  // Final time
    double q0;  // Initial position
    double qf;  // Final position
    double v0;  // Initial velocity
    double vf;  // Final velocity
    double a0;  // Initial acceleration
    double af;  // Final acceleration
};

int main() {
    // Example usage
    double initial_time = 0.0;
    double final_time = 2.0;
    double initial_force = 0.0;
    double final_force = 49.0;
    double initial_velocity = 0.0;
    double final_velocity = 0.0;
    double initial_acceleration = 0.0;
    double final_acceleration = 0.0;

    // Create a FourthOrderTrajectoryGenerator object
    FourthOrderTrajectoryGenerator generator(initial_time, final_time,
                                             initial_force, final_force,
                                             initial_velocity, final_velocity,
                                             initial_acceleration, final_acceleration);

    // Generate and print the trajectory at different time points
    for (double t = initial_time; t <= final_time; t += 0.5) {
        double position = generator.generateTrajectory(t);
        std::cout << "Time: " << t << ", Position: " << position << std::endl;
    }

    // Visualization using Matplotlibcpp
    std::vector<double> times;
    std::vector<double> positions;

    // Populate vectors for plotting
    for (double t = initial_time; t <= final_time; t += 0.1) {
        double position = generator.generateTrajectory(t);
        times.push_back(t);
        positions.push_back(position);
    }

    // Plot the trajectory
    matplotlibcpp::plot(times, positions);
    matplotlibcpp::title("4th Order Polynomial Trajectory");
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("Position");
    matplotlibcpp::show();
    return 0;
}



#endif //EE3100704_PROJECTS_FOURTHORDERTRAJECTORYGENERATOR_H
