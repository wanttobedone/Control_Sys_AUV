#include "rov_localization/camera_calibrator.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv)
{
  const cv::String keys = "{help h||显示帮助}"
                          "{video v||输入录像路径（只读）}"
                          "{config c||calibration_options.yaml路径}"
                          "{output o||尚不存在的新结果目录，父目录须存在}"
                          "{tag-id||覆盖配置的目标ID；-1自动选择，未提供时保留YAML值}";
  cv::CommandLineParser parser(argc, argv, keys);
  if (argc == 1 || parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }
  try
  {
    auto options = rov_localization::loadOptions(parser.get<std::string>("config"));
    if (parser.has("tag-id"))
    {
      options.target_id = parser.get<int>("tag-id");
    }
    const auto video = parser.get<std::string>("video");
    const auto output = parser.get<std::string>("output");
    if (!parser.check() || video.empty() || output.empty())
    {
      parser.printErrors();
      throw std::invalid_argument("需要--video=路径 --config=路径 --output=新目录");
    }
    rov_localization::CameraCalibrator calibrator(options);
    const auto result = calibrator.run(video, output);
    std::cout << result.message << '\n';
    std::cout << "报告目录：" << std::filesystem::absolute(output).string() << '\n';
    if (!result.success)
    {
      std::cerr << "本次未生成camera_wet.xml；请查看calibration_report.yaml的message和tags\n";
      return 2;
    }
    std::cout << "候选参数：camera_wet.xml，拟合RMS=" << result.rms_px
              << "px，留出RMS=" << result.holdout_rms_px << "px\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "标定工具失败：" << error.what() << '\n';
    return 1;
  }
}
