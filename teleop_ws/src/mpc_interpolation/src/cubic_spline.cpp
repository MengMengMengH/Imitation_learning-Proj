#include "../include/mpc_interpolation/cubic_spline.hpp"
#include <iostream>
#include <chrono>
#include <random>
#include <QApplication>
#include <QMainWindow>
#include <QChart>
#include <QLineSeries>
#include <QScatterSeries>
#include <QChartView>
#include <QPainter>


int main(int argc, char **argv) 
{
    QApplication app(argc, argv);
    QMainWindow window;

    QChart *chart = new QChart();
    QLineSeries *lineSeries = new QLineSeries();
    QScatterSeries *scatterSeries = new QScatterSeries();
    QScatterSeries *referencePoints = new QScatterSeries();

    referencePoints->setMarkerSize(10); // 放大参考点
    referencePoints->setColor(Qt::red); // 设置颜色为红色

    const int num_segs = 5;
    const int num_points = 50;
    const double delta = 4.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-delta, delta);


    Eigen::VectorXd start_pos(7);
    Eigen::VectorXd start_vel(7);
    Eigen::VectorXd start_acc(7);
    Eigen::VectorXd end_pos(7);
    start_pos << 0, 0, 0, 0, 0, 0, 0; // 起始位置
    start_vel << 0, 0, 0, 0, 0, 0, 0; // 起始速度

    end_pos << 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15; // 终点位置


    for(int seg = 0;seg<num_segs; seg++)
    {   
        CubicSplineTrajectoryPlanner planner(start_pos, start_vel, end_pos, num_points);
        try {
            auto start = std::chrono::high_resolution_clock::now();
            auto trajectory = planner.generateTrajectory();
            // auto coeffs = planner.computeCubicCoefficients();
            auto end = std::chrono::high_resolution_clock::now();
            auto solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            for (int i = 0; i < num_points; ++i) {
                double t = static_cast<double>(i) / num_points + seg;
                std::cout << "t: " << t << std::endl;
                double y = trajectory[i](0); // 可视化第一个维度的数据
                lineSeries->append(t, y);
                scatterSeries->append(t, y);
            }
            referencePoints->append(seg + 1, end_pos(0));
            // std::cout << "Coefficients: "<< planner.coeffs << std::endl;
            if (seg < num_segs - 1) 
            {
                // Update start conditions for next segment
                start_pos = end_pos;
                start_vel = planner.evaluatePolynomial(1.0, planner.coeffs, 1);
                // Randomly adjust end_pos for next segment
                for (int i = 0; i < 7; ++i) {
                    end_pos[i] += dis(gen);
                }
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    chart->addSeries(lineSeries);
    chart->addSeries(scatterSeries);
    chart->addSeries(referencePoints);
    chart->createDefaultAxes();

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setRubberBand(QChartView::RectangleRubberBand);

    window.setCentralWidget(chartView);
    window.resize(800, 600);
    window.show();

    return app.exec();
    // return 0;
}