#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
constexpr double MY_PI = 3.1415926;


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    float angleToPi = rotation_angle / 180.0f * acos(-1);
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // roate by z
    // model << std::cos(angleToPi), -std::sin(angleToPi), 0, 0,
    //         std::sin(angleToPi), std::cos(angleToPi), 0, 0,
    //         0, 0, 1, 0,
    //         0, 0, 0, 1;

    // rotate by x
    // model << 1, 0, 0, 0,
    //     0, std::cos(angleToPi), -std::sin(angleToPi), 0,
    //     0, std::sin(angleToPi), std::cos(angleToPi), 0,
    //     0, 0, 0, 1;

    // rotate by y
    model << std::cos(angleToPi), 0, std::sin(angleToPi), 0,
        0, 1, 0, 0,
        -std::sin(angleToPi), 0, std::cos(angleToPi), 0,
        0, 0, 0, 1;
    

    return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {
    // Rodrigues's Rotation Formula
    Eigen::Matrix3f model = Eigen::Matrix3f::Identity();
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    Eigen::Matrix3f matrixN = Eigen::Matrix3f::Identity();
    matrixN << 0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0; 
    Eigen::RowVector3f axisT = axis.transpose();
    model = cosAngle * model + (1 - cosAngle) * (axis * axisT) + sinAngle * matrixN;
    Eigen::Matrix4f resultMatrix = Eigen::Matrix4f::Identity();
    resultMatrix << model(0, 0), model(0, 1), model(0, 2), 0,
                model(1, 0), model(1, 1), model(1, 2), 0,
                model(2, 0), model(2, 1), model(2, 2), 0,
                0, 0, 0, 1; 
    return resultMatrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection << 1 / (aspect_ratio * std::tan(eye_fov / 2)), 0, 0, 0,
                0, 1 / (std::tan(eye_fov/2)), 0, 0,
                0, 0, -(zFar + zNear)/(zFar - zNear), -(2 * zFar * zNear) / (zFar - zNear),
                0, 0, 1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 30;
    bool command_line = false;
    std::string filename = "output.png";
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }
    std::cout << "check get_rotation result" << std::endl;
    std::cout << get_model_matrix(30) << std::endl;
    std::cout << get_rotation(Eigen::Vector3f(1, 0, 0), -30) << std::endl;

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    int esc_key = 27;
    while (key != esc_key) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
