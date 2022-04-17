//   switch (robo_inf.catch_cube_mode_status.load())
//   {
// /****************angle*********************************/
//     case CatchMode::spin:
//     {
//       cv::putText(src_img,
//               "spin mode",
//               cv::Point(50, 50),
//               cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
//       pnp_angle.y = src_img.cols * 0.57 - (temp_rect_ .x + temp_rect_.width * 0.5);
//       if (cube_middle_detect_times < cube_targeted_detect_flag_times)
//       {
//         if (pnp_angle.y < cube_target_yaw_angle_errors_range &&
//             pnp_angle.y > -cube_target_yaw_angle_errors_range)
//         {
//           cube_middle_detect_times++;
//         }
//         else
//         {
//           RoboSpinCmdUartBuff uart_temp_spin_cmd;
//           uart_temp_spin_cmd.yaw_angle = pnp_angle.y;
//           std::cout << "uart_temp_spin_cmd.yaw_angle" << uart_temp_spin_cmd.yaw_angle << "\n";
//           serial_robot_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
//         }
//       }
//       else
//       {
//         RoboSpinCmdUartBuff uart_temp_spin_cmd;
//         uart_temp_spin_cmd.yaw_angle = 0.f;
//         for (int i = 0; i < 3; i++)
//         {
//           serial_robot_->write((uint8_t *)&uart_temp_spin_cmd, sizeof(uart_temp_spin_cmd));
//           std::cout << "send yaw 0\n";
//           usleep(cube_target_echo_uart_cmd_sleep_time);
//         }
//         cube_middle_detect_times = 0;
//         robo_inf.catch_cube_mode_status.store(CatchMode::go);
//       }
//       break;
//     }

// /***************direct********************************/
//     case CatchMode::go:
//     {
//       cv::putText(src_img,
//                   "go straight mode",
//                   cv::Point(50, 50),
//                   cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
//       // judge the cube if in the center of frame
//       // sand the message in ten times
//       if (cude_front_detect_times < cube_targeted_detect_flag_times)
//       {
//         cv::Rect object_rect(temp_rect_.x + temp_rect_.width *0.5 - 50,
//                              temp_rect_.y + temp_rect_.height - 100,
//                              100, 100); // 底部与目标重叠且居中，固定大小的 temp_rect_ 用以 pnp

//         float select_cube_dis = cube_target_distance_offset - pnp_angle.x;
//         // if the angles in around the range
//         if (select_cube_dis < cube_target_distance_errors_range &&
//             select_cube_dis > -cube_target_distance_errors_range)
//         {
//           cude_front_detect_times++;
//         }
//         else
//         {
//           RoboGoCmdUartBuff uart_temp_go_cmd;
//           uart_temp_go_cmd.distance = select_cube_dis;
//           std::cout << "uart_temp_go_cmd.distance:" << uart_temp_go_cmd.distance << "\n";
//           serial_robot_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
//         }
//       }
//       else
//       {
//         // send  0 meters message for the Electronically controlled then the robo ready to  catch the cube for three time
//         RoboGoCmdUartBuff uart_temp_go_cmd;
//         uart_temp_go_cmd.distance = 0.f;
//         std::cout << "cathch_mode" << std::endl;
//         for (int i = 0; i < 3; i++)
//         {
//           serial_robot_->write((uint8_t *)&uart_temp_go_cmd, sizeof(uart_temp_go_cmd));
//           std::cout << "send stop \n";
//           usleep(cube_target_echo_uart_cmd_sleep_time);
//         }
//         robo_inf.catch_cube_mode_status.store(CatchMode::catch_cube);
//         cude_front_detect_times = 0;
//       }
//       break;
//     }

// /***************catch******************************/
//     case CatchMode::catch_cube:
//     {
//       std::cout << temp_rect_.area() << std::endl;
//       cv::putText(src_img,
//                   "catch cube mode",
//                   cv::Point(50, 50),
//                   cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
//       RoboCatchCmdUartBuff uart_temp_catch_cmd;
//       // make the state of the cube 
//       if (detected_objects.at(0).id == 0 ||
//           detected_objects.at(0).id == 3)
//       {
//         uart_temp_catch_cmd.cube_state = CUBE_UP;
//       }
//       else if (detected_objects.at(0).id == 1 ||
//                detected_objects.at(0).id == 4)
//       {
//         uart_temp_catch_cmd.cube_state = CUBE_DOWN;
//       }
//       else if (detected_objects.at(0).id == 2 ||
//                detected_objects.at(0).id == 5)
//       {
//         uart_temp_catch_cmd.cube_state = CUBE_STAND;
//       }
//       // standing mode judge the age
//       if(uart_temp_catch_cmd.cube_state == CUBE_STAND) {
//       // get the type of the cube by judging that area
//         if (temp_rect_.area() > cube_1_min_area &&
//             temp_rect_.area() < cube_1_max_area)
//         {
//           uart_temp_catch_cmd.cube_type = CUBE_1;
//         }

//         if (temp_rect_.area() > 21000 &&
//             temp_rect_.area() < 33000)
//         {
//           uart_temp_catch_cmd.cube_type = CUBE_2;
//         }
//         else if (temp_rect_.area() > cube_3_min_area &&
//                 temp_rect_.area() < cube_3_max_area)
//         {
//           uart_temp_catch_cmd.cube_type = CUBE_3;
//         }
//         else if (temp_rect_.area() > cube_4_min_area &&
//                 temp_rect_.area() < cube_4_max_area)
//         {
//           uart_temp_catch_cmd.cube_type = CUBE_4;
//         }
//         if (uart_temp_catch_cmd.cube_state == CUBE_STAND)
//         {
//           uart_temp_catch_cmd.cube_type = CUBE_UNCERTAIN;
//         }
//       }
//       // get the type of the cube by judging that area
//       if (temp_rect_.area() > cube_1_min_area &&
//           temp_rect_.area() < cube_1_max_area)
//       {
//         uart_temp_catch_cmd.cube_type = CUBE_1;
//       }

//       if (temp_rect_.area() > cube_2_min_area &&
//           temp_rect_.area() < cube_2_max_area)
//       {
//         uart_temp_catch_cmd.cube_type = CUBE_2;
//       }
//       else if (temp_rect_.area() > cube_3_min_area &&
//                temp_rect_.area() < cube_3_max_area)
//       {
//         uart_temp_catch_cmd.cube_type = CUBE_3;
//       }
//       else if (temp_rect_.area() > cube_4_min_area &&
//                temp_rect_.area() < cube_4_max_area)
//       {
//         uart_temp_catch_cmd.cube_type = CUBE_4;
//       }
//       if (uart_temp_catch_cmd.cube_state == CUBE_STAND)
//       {
//         uart_temp_catch_cmd.cube_type = CUBE_UNCERTAIN;
//       }
//       // send the true message to the serial for three times
//       if (uart_temp_catch_cmd.cube_type != 0x00)
//       {
//         for (int i = 0; i < 3; i++)
//         {
//           serial_robot_->write((uint8_t *)&uart_temp_catch_cmd, sizeof(uart_temp_catch_cmd));
//           std::cout << "catch, temp_rect_ size:" << temp_rect_.area() << "  temp_rect_ type:"
//                     << (int)uart_temp_catch_cmd.cube_type << "  temp_rect_ state:"
//                     << (int)uart_temp_catch_cmd.cube_state << "\n";
//           usleep(cube_target_echo_uart_cmd_sleep_time);
//         }
//       }
//       robo_inf.catch_cube_mode_status.store(CatchMode::detect_mode);
//       break;
//     }
// /*******************detect_**************************/
//     case CatchMode::detect_mode:
//     {
//     cv::putText(src_img,
//                 "detect cube mode",
//                 cv::Point(50, 50),
//                 cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 150, 255), 1);
//       RoboCubeStateUartBuff uart_temp_state_cmd;
//       if (detected_objects.at(0).id == 0 ||
//           detected_objects.at(0).id == 3)
//       {
//         uart_temp_state_cmd.cube_status = CUBE_UP;
//       }
//       else if (detected_objects.at(0).id == 1 ||
//                detected_objects.at(0).id == 4)
//       {
//         uart_temp_state_cmd.cube_status = CUBE_DOWN;
//       }
//       else if (detected_objects.at(0).id == 2 ||
//                detected_objects.at(0).id == 5)
//       {
//         uart_temp_state_cmd.cube_status = CUBE_STAND;
//       }
//       uart_temp_state_cmd.cube_status = CUBE_DOWN;
//       uart_temp_state_cmd.cube_type = 0x03;
//       if (uart_temp_state_cmd.cube_status != 0x00)
//       {
//       for (int i = 0; i < 3; i++)
//         {
//           serial_robot_->write((uint8_t *)&uart_temp_state_cmd, sizeof(uart_temp_state_cmd));
//           std::cout << (int)uart_temp_state_cmd.cube_type << "  temp_rect_ state:"
//                     << (int)uart_temp_state_cmd.cube_status << "\n";
//           usleep(cube_target_echo_uart_cmd_sleep_time);
//         }
//       }
//       robo_inf.catch_cube_mode_status.store(CatchMode::off);

//       break;
//     }
//     default:
//       break;
//   }    