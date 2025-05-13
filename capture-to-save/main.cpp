// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

#include "example.hpp"          // Include short list of convenience functions for rendering
#include "imgui.h"

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char* argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start(cfg);

    // 等待自动曝光稳定
    for (auto i = 0; i < 30; ++i)
        pipe.wait_for_frames();

    bool snap_requested = false;
    while (app) // Application still alive?
    {
        // 1. 渲染ImGui按钮
        if (ImGui::Begin("Control Panel"))
        {
            if (ImGui::Button("Snap"))
            {
                snap_requested = true;
            }
        }
        ImGui::End();
        rs2::frameset data = pipe.wait_for_frames();    // Wait for next set of frames from the camera

        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        app.show(data.apply_filter(printer).    // Print each enabled stream frame rate
                      apply_filter(color_map)); // Find and colorize the depth data

		// Save the last frame to disk
        // 2. 如果点击了snap按钮，保存color和depth图像
        if (snap_requested)
        {
            // 保存color图像
            auto color_frame = data.get_color_frame();
            if (color_frame)
            {
                std::stringstream color_file;
                color_file << "snap-color-" << color_frame.get_frame_number() << ".png";
                stbi_write_png(color_file.str().c_str(), color_frame.get_width(), color_frame.get_height(),
                    color_frame.get_bytes_per_pixel(), color_frame.get_data(), color_frame.get_stride_in_bytes());
                std::cout << "Saved " << color_file.str() << std::endl;

                //保存 meta 数据
				std::stringstream csv_file;
				csv_file << "snap-color-" << color_frame.get_frame_number() << "-metadata.csv";
				metadata_to_csv(color_frame, csv_file.str());
				std::cout << "Saved " << csv_file.str() << std::endl;
            }

            // 保存depth图像（原始）
			auto depth_frame = data.get_depth_frame();
			if (depth_frame)
			{
				std::stringstream depth_file;
				depth_file << "snap-depth-" << depth_frame.get_frame_number() << ".png";
				stbi_write_png(depth_file.str().c_str(), depth_frame.get_width(), depth_frame.get_height(),
					depth_frame.get_bytes_per_pixel(), depth_frame.get_data(), depth_frame.get_stride_in_bytes());
				std::cout << "Saved " << depth_file.str() << std::endl;

                // 彩色化depth
                auto depth_colorized = color_map.process(depth_frame);
                auto vf = depth_colorized.as<rs2::video_frame>();
                depth_file.clear();
                depth_file << "snap-depth-color" << depth_frame.get_frame_number() << ".png";
                stbi_write_png(depth_file.str().c_str(), vf.get_width(), vf.get_height(),
                    vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                std::cout << "Saved " << depth_file.str() << std::endl;

                //保存 meta 数据
				std::stringstream csv_file;
				csv_file << "snap-depth-" << depth_frame.get_frame_number() << "-metadata.csv";
				metadata_to_csv(depth_frame, csv_file.str());
				std::cout << "Saved " << csv_file.str() << std::endl;
			}

            snap_requested = false; // 重置
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
