#ifndef _POSE_ESTIMATOR_H_
#define _POSE_ESTIMATOR_H_

/*
 Pose Estimator calculates the global sphere positions from the circles data. Camera gemoetry is needed.
 ... calculates the robot pose via an iterative process.
*/

#include <math.h>

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "ceres/rotation.h"

#include "Pose.h"
#include "Sphere.h"


struct ErrorModel
{
	ErrorModel(Eigen::Vector3f model_v, Eigen::Vector3f measurement_v): model_v(model_v), measurement_v(measurement_v) { };

	template <typename T> bool operator() (const T* const pose, T* residual) const
	{
		T model[3] = { T(model_v(0)), T(model_v(1)), T(model_v(2)) };
		T measurement[3] = { T(measurement_v(0)), T(measurement_v(1)), T(measurement_v(2)) };

		T euler[3] = {pose[3], pose[4], pose[5]};
		T rotation_matrix[9];
		ceres::EulerAnglesToRotationMatrix(euler, 3, rotation_matrix);

		T model_rotated[3];
		dot(rotation_matrix, model, model_rotated);

		residual[0] = pose[0] + model_rotated[0] - measurement[0];
		residual[1] = pose[1] + model_rotated[1] - measurement[1];
		residual[2] = pose[2] + model_rotated[2] - measurement[2];
		return true;
	}

	template <typename T> void dot(const T* const matrix, const T* const vector, T* result) const
	{
		result[0] = matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2];
		result[1] = matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2];
		result[2] = matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2];
	}

private:
	Eigen::Vector3f model_v;
	Eigen::Vector3f measurement_v;
};


class PoseEstimator
{
public:
    PoseEstimator() {};

    Pose getPoseFromModel(std::vector<Sphere> model, std::vector<Sphere> measurements)
    {
        // Find all colors of measurement in model and bring them to corresponding order
        std::vector<Sphere> compare_model;
		for (auto measurement_sphere : measurements)
		{
			for (auto model_sphere : model)
			{
				if (measurement_sphere.color == model_sphere.color)
                {
                    compare_model.push_back(model_sphere);
                    break;
                }
			}
		}

        Eigen::VectorXd vector = Eigen::VectorXd::Zero(6);

        ceres::Problem problem;
        for (int i = 0; i < measurements.size(); i++)
        {
            Sphere sphere_model = compare_model.at(i);
            Sphere sphere_measurement = measurements.at(i);

            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ErrorModel, 3, 6>(
				new ErrorModel(sphere_model.position, sphere_measurement.position)
			);
            problem.AddResidualBlock(cost_function, nullptr, vector.data());
        }

        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // cout << summary.FullReport() << endl;

        Pose pose = Pose();
		pose.position(X) = vector(0);
		pose.position(Y) = vector(1);
		pose.position(Z) = vector(2);
		pose.orientation(ROLL) = vector(3) * 180. / M_PI;
		pose.orientation(PITCH) = vector(4) * 180. / M_PI;
		pose.orientation(YAW) = vector(5) * 180. / M_PI;
        return pose;
    }
};

#endif /* _POSE_ESTIMATOR_H_ */
