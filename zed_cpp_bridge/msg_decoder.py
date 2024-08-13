import flatbuffers
from .aruco_msgs_generated import StereoImageMarkers, ImageMarkers, Marker, Point2D


class MarkerDecoder:

    def __init__(self):
        pass

    def decode(self, buf):
        """Decode the buffer into a structured dictionary format."""
        if not StereoImageMarkers.StereoImageMarkersBufferHasIdentifier(buf, 0):
            raise ValueError("Buffer does not have the correct FlatBuffers identifier (ODET).")

        stereo_image_markers = StereoImageMarkers.GetRootAsStereoImageMarkers(buf, 0)

        # Process left and right image markers
        left_markers_info = self.process_image_markers(stereo_image_markers.LeftImage())
        right_markers_info = self.process_image_markers(stereo_image_markers.RightImage())

        return {"left": left_markers_info, "right": right_markers_info}

    def process_image_markers(self, image):
        """Process markers for the given image and return as a dictionary."""
        markers_info = {
            "image_name": "",
            "markers": []
        }

        if image:
            markers_info["image_name"] = image.ImageName().decode('utf-8')  # Convert bytes to string
            num_markers = image.MarkersLength()

            for i in range(num_markers):
                marker = image.Markers(i)
                marker_info = {
                    "id": marker.Id(),
                    "corners": [{"x": marker.Corners(j).X(), "y": marker.Corners(j).Y()} for j in range(marker.CornersLength())]
                }
                markers_info["markers"].append(marker_info)

        return markers_info
