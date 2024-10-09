#ifndef IMAGE_OVERLAY_HPP
#define IMAGE_OVERLAY_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

/**
 * @class ImageOverlay
 * @brief A ROS2 Node that overlays a foreground PGM image onto a background PNG image.
 * 
 * The class loads a PNG and PGM image, filters and processes the foreground image, 
 * and overlays it onto the background image. The result is displayed and saved as an output image.
 */
class ImageOverlay : public rclcpp::Node
{
public:
    /**
     * @brief Constructs an ImageOverlay object and initializes the node.
     * 
     * Loads background and foreground images, resizes the foreground if necessary, 
     * and performs the overlay operation.
     */
    ImageOverlay();

private:
    /**
     * @brief Holds the background image (PNG) loaded with RGBA channels.
     */
    cv::Mat background_;

    /**
     * @brief Holds the foreground image (PGM), filtered and processed as RGBA.
     */
    cv::Mat foreground_;

    /**
     * @brief Loads a PGM image, rotates it 90 degrees counter-clockwise, 
     * and converts it into an RGBA image with non-black pixels made transparent.
     * 
     * @param input_pgm The file path to the input PGM image.
     * @return A processed RGBA image.
     */
    cv::Mat map(const std::string& input_pgm);

    /**
     * @brief Overlays the foreground image onto the background image using alpha blending.
     * 
     * The function blends the RGB channels of the background and foreground images 
     * based on the alpha channel of the foreground (PGM image).
     * The resulting composited image is displayed and saved.
     */
    void overlayImages();
};

#endif // IMAGE_OVERLAY_HPP
