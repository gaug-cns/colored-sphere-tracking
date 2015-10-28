#ifndef _SPHERE_FILTERS_H_
#define _SPHERE_FILTERS_H_

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "ceres/rotation.h"

#include "helper.h"


#define SIGMA_ACCURACY 4
#define PI 3.1415



class LikelihoodField : public ceres::FirstOrderFunction
{
    
public:
    std::vector<Measurement> measurements;
    
    LikelihoodField(std::vector<Measurement> measurements) : measurements(measurements) { }
    
    virtual bool Evaluate(const double* parameters, double* cost, double* gradient) const
    {
        const double x = parameters[0];
        const double y = parameters[1];
        const double z = parameters[2];
        
        cost[0] = 0;
        gradient[0] = 0;
        gradient[1] = 0;
        gradient[2] = 0;
        for (auto m : measurements)
        {
            const double norm = sqrt( SQUARE(x - m.mean(0)) + SQUARE(y - m.mean(1)) + SQUARE(z - m.mean(2)) );
            if ( norm < SIGMA_ACCURACY * m.sigma)
            {
                double temp = m.maximum * exp( - (SQUARE(x - m.mean(0)) + SQUARE(y - m.mean(1)) + SQUARE(z - m.mean(2))) / (2 * SQUARE(m.sigma)) );
                cost[0] -= temp;
                gradient[0] -= (m.mean(0) - x) / SQUARE(m.sigma) * temp;
                gradient[1] -= (m.mean(1) - y) / SQUARE(m.sigma) * temp;
                gradient[2] -= (m.mean(2) - z) / SQUARE(m.sigma) * temp;
            }
        }
        return true;
    }
    
    virtual int NumParameters() const { return 3; }
};


class LikelihoodFilter
{
    
private:
    std::vector<Measurement> history;
    
    Eigen::Vector3f position;
    float certainty;
    
    Color color;
    
    void saveLocalMaximum(Eigen::Vector3f start_position)
    {
        Eigen::Vector3f temp_position;
        float temp_certainty;
        findLocalMaximum(start_position, temp_position, temp_certainty);
        position = temp_position;
        certainty = temp_certainty;
    }
    
    void findLocalMaximum(Eigen::Vector3f start_position, Eigen::Vector3f& final_position, float& final_certainty)
    {
        LikelihoodField *likelihood_field = new LikelihoodField(history);
        ceres::GradientProblem problem(likelihood_field);
        ceres::GradientProblemSolver::Options options;
        ceres::GradientProblemSolver::Summary summary;
        
        double parameters[3] = { start_position(0), start_position(1), start_position(2)};
        double cost[1];
        double gradient[1];
        ceres::Solve(options, problem, parameters, &summary);
        likelihood_field->Evaluate(parameters, cost, gradient);
        
        Eigen::Vector3f result;
        result << parameters[0], parameters[1], parameters[2];
        final_position = result;
        final_certainty = -cost[0];
    }
    
    
public:
    LikelihoodFilter(Color color) : color(color)
    {
        history = {};
        
        position = Eigen::Vector3f::Zero();
        certainty = 0.;
    }
    
    void update(std::vector<Measurement> measurements, float time)
    {
        // Init calculation
        for (auto m : measurements)
        {
            m.calculate(time);
            history.push_back(m);
        }
        
        // Remove old (unimportant) entries
        for (int i = 0; i < history.size(); i++)
        {
            if (history.at(i).time < time - 100) // [ms]
            {
                history.erase(history.begin() + i);
                i--;
            }
        }
        
        certainty *= 0.9;
        if (history.size() > 0)
            findGlobalMaximum(time);
    }
    
    void findGlobalMaximum(float time)
    {
        std::sort(history.begin(), history.end());
        float initial_maximum = history.back().maximum;

        // Find initial positions (1st round)
        float sum_maximas = 0;
        int index_sum_maximum = 0;
        for (int i = 0; i < history.size(); i++)
        {
            Measurement m = history[i];
            sum_maximas += m.maximum;
            if (sum_maximas > initial_maximum)
            {
                index_sum_maximum = i;
                break;
            }
        }
        std::vector<Measurement> initials(history.begin() + index_sum_maximum, history.end());
        if (initials.size() == 1)
        {
            saveLocalMaximum( initials.back().mean );
            return;
        }

        // Rectangular approximation (2nd round)
        for (int i = 0; i < initials.size(); i++)
        {
            Measurement m = initials.at(i);
            float sum_rect = m.maximum;
            for (int j = 0; j < (history.size() - initials.size()) + i; j++)
            {
                Measurement m_j = history.at(j);
                if ((m_j.mean - m.mean).norm() < SIGMA_ACCURACY * m_j.sigma)
                    sum_rect += m_j.maximum;
            }
            if (sum_rect < initial_maximum)
            {
                initials.erase(initials.begin() + i);
                i--;
            }
        }
        if (initials.size() == 1)
        {
            saveLocalMaximum( initials.back().mean );
            return;
        }
        
        saveLocalMaximum( initials.back().mean );
        return;
        
        /* // Test all remaining initial positions manually
        Eigen::Vector3f best_position;
        float maximum = initial_maximum;
        for (auto m : initials)
        {
            Eigen::Vector3f temp_position;
            float temp_certainty;
            findLocalMaximum(m.mean, temp_position, temp_certainty);

            if (temp_certainty > maximum)
            {
                best_position = temp_position;
                maximum = temp_certainty;
            }
        }
        position = best_position;
        certainty = maximum; */
    }

    Sphere getSphere() { return Sphere(position, color); }
    
    bool isTracked() { return (certainty > 1.0); }
};


class SphereFilters
{

private:
    LikelihoodFilter *red_filter;
    LikelihoodFilter *blue_filter;
    LikelihoodFilter *green_filter;
    LikelihoodFilter *yellow_filter;
    
    
public:
    SphereFilters()
    {
        red_filter = new LikelihoodFilter(Red);
        blue_filter = new LikelihoodFilter(Blue);
        green_filter = new LikelihoodFilter(Green);
        yellow_filter = new LikelihoodFilter(Yellow);
    }
    
    void update(std::vector<Measurement> measurements, float time)
    {
        std::vector<Measurement> red_measurements, blue_measurements, green_measurements, yellow_measurements;
        
        for (auto m : measurements) {
            switch (m.color) {
                case Red: red_measurements.push_back(m); break;
                case Blue: blue_measurements.push_back(m); break;
                case Green: green_measurements.push_back(m); break;
                case Yellow: yellow_measurements.push_back(m); break;
            }
        }
        
        red_filter->update(red_measurements, time);
        blue_filter->update(blue_measurements, time);
        green_filter->update(green_measurements, time);
        yellow_filter->update(yellow_measurements, time);
    }
    
    std::vector<Sphere> getTrackedSpheres(float time)
    {
        std::vector<Sphere> result;
        
        if (red_filter->isTracked())
            result.push_back( red_filter->getSphere() );
        
        if (blue_filter->isTracked())
            result.push_back( blue_filter->getSphere() );
        
        if (green_filter->isTracked())
            result.push_back( green_filter->getSphere() );
        
        if (yellow_filter->isTracked())
            result.push_back( yellow_filter->getSphere() );
        
        return result;
    }
};

#endif