from qgis.core import *
from qgis.utils import iface
from PyQt5.QtCore import QVariant
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QComboBox, QPushButton, QLineEdit, QMessageBox, QHBoxLayout, QFileDialog, QCheckBox
import os
import time
from pathlib import Path

class LayerSelectionDialog(QDialog):
    def __init__(self, parent=None):
        super(LayerSelectionDialog, self).__init__(parent)
        self.setWindowTitle("Select Layers and Output File")
        self.resize(500, 400)
        
        # Create layout
        layout = QVBoxLayout()
        
        # Get all vector layers
        self.vector_layers = [layer for layer in QgsProject.instance().mapLayers().values() 
                             if isinstance(layer, QgsVectorLayer)]
        layer_names = [layer.name() for layer in self.vector_layers]
        
        # Meter layer selection
        layout.addWidget(QLabel("Select Meter Layer:"))
        self.meter_combo = QComboBox()
        self.meter_combo.addItems(layer_names)
        layout.addWidget(self.meter_combo)
        
        # Pipe layer selection
        layout.addWidget(QLabel("Select Pipe Layer:"))
        self.pipe_combo = QComboBox()
        self.pipe_combo.addItems(layer_names)
        layout.addWidget(self.pipe_combo)
        
        # Node layer selection (ใหม่)
        layout.addWidget(QLabel("Select Node Layer (optional):"))
        self.node_combo = QComboBox()
        self.node_combo.addItems(["[None]"] + layer_names)
        layout.addWidget(self.node_combo)
        
        # Road layer selection
        layout.addWidget(QLabel("Select Road Layer:"))
        self.road_combo = QComboBox()
        self.road_combo.addItems(layer_names)
        layout.addWidget(self.road_combo)
        
        # Output file path with browse button
        layout.addWidget(QLabel("Output File Path:"))
        
        # Create horizontal layout for output path and browse button
        output_layout = QHBoxLayout()
        
        # Default output path
        user_home = str(Path.home())
        default_output = os.path.join(user_home, "QGIS_Output", "spider_output.gpkg")
        
        # Add output path text field
        self.output_path = QLineEdit(default_output)
        output_layout.addWidget(self.output_path, 3)  # Give more stretch to the text field
        
        # Add browse button
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.browse_output_file)
        output_layout.addWidget(self.browse_button, 1)  # Give less stretch to the button
        
        # Add the horizontal layout to the main layout
        layout.addLayout(output_layout)
        
        # Ensure directory exists for default path
        os.makedirs(os.path.dirname(default_output), exist_ok=True)
        
        # Maximum connection distance setting
        layout.addWidget(QLabel("Maximum Connection Distance (map units):"))
        self.max_distance_input = QLineEdit("50")  # Default 50 map units
        layout.addWidget(self.max_distance_input)
        
        # Node preference settings (ใหม่)
        layout.addWidget(QLabel("Node Priority Settings:"))
        node_settings_layout = QVBoxLayout()
        
        # Add preference for connecting to nodes
        self.prefer_nodes_checkbox = QCheckBox("Prefer connecting to pipe nodes/endpoints")
        self.prefer_nodes_checkbox.setChecked(True)
        node_settings_layout.addWidget(self.prefer_nodes_checkbox)
        
        # Maximum distance from pipe endpoint to consider
        node_distance_layout = QHBoxLayout()
        node_distance_layout.addWidget(QLabel("Max distance from pipe nodes:"))
        self.node_distance_input = QLineEdit("15")  # Default 15 map units
        node_distance_layout.addWidget(self.node_distance_input)
        node_settings_layout.addLayout(node_distance_layout)
        
        layout.addLayout(node_settings_layout)
        
        # Button to run script
        self.run_button = QPushButton("Run Script")
        self.run_button.clicked.connect(self.run_script)
        layout.addWidget(self.run_button)
        
        self.setLayout(layout)
    
    def browse_output_file(self):
        """Open file dialog to select output file location"""
        # Get the current directory from the text field
        current_dir = os.path.dirname(self.output_path.text())
        
        # Make sure it exists
        if not os.path.exists(current_dir):
            current_dir = str(Path.home())
        
        # Open file dialog
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Select Output File Location",
            current_dir,
            "GeoPackage (*.gpkg);;All Files (*.*)"
        )
        
        # Update text field if a file was selected
        if file_path:
            # Add .gpkg extension if not provided
            if not file_path.lower().endswith('.gpkg'):
                file_path += '.gpkg'
            self.output_path.setText(file_path)
    
    def run_script(self):
        try:
            # Get selected layers
            meter_layer = self.vector_layers[self.meter_combo.currentIndex()]
            pipe_layer = self.vector_layers[self.pipe_combo.currentIndex()]
            road_layer = self.vector_layers[self.road_combo.currentIndex()]
            output_path = self.output_path.text()
            
            # Get optional node layer if selected
            node_layer = None
            if self.node_combo.currentIndex() > 0:  # First item is [None]
                node_layer = self.vector_layers[self.node_combo.currentIndex() - 1]
            
            # Make sure output directory exists
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            
            # Get max distance values
            try:
                max_distance = float(self.max_distance_input.text())
                if max_distance <= 0:
                    raise ValueError("Distance must be positive")
                
                node_max_distance = float(self.node_distance_input.text())
                if node_max_distance <= 0:
                    raise ValueError("Node distance must be positive")
            except ValueError as e:
                QMessageBox.warning(self, "Invalid Distance", "Please enter valid positive numbers for distance values.")
                return
                
            # Run the main script
            self.run_meter_pipe_matching(
                meter_layer, 
                pipe_layer, 
                road_layer, 
                output_path, 
                max_distance,
                self.prefer_nodes_checkbox.isChecked(),
                node_max_distance,
                node_layer
            )
            
            self.accept()  # Close dialog when done
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error running script: {str(e)}")
    
    def run_meter_pipe_matching(self, meter_layer, pipe_layer, road_layer, output_path, 
                                max_distance, prefer_nodes=True, node_max_distance=15, node_layer=None):
        # Print layer information
        print(f"Using meter layer: {meter_layer.name()}")
        print(f"Using pipe layer: {pipe_layer.name()}")
        print(f"Using road layer: {road_layer.name()}")
        if node_layer:
            print(f"Using node layer: {node_layer.name()}")
        print(f"Output will be saved to: {output_path}")
        print(f"Maximum connection distance: {max_distance}")
        print(f"Prefer pipe nodes/endpoints: {prefer_nodes}")
        print(f"Maximum node distance: {node_max_distance}")
        
        # Create spatial indices for better performance
        pipe_index = QgsSpatialIndex(pipe_layer.getFeatures())
        road_index = QgsSpatialIndex(road_layer.getFeatures())
        
        # Create node index if node layer provided
        node_index = None
        if node_layer:
            node_index = QgsSpatialIndex(node_layer.getFeatures())
            print("Created spatial index for node layer")
        
        # Add PIPE_ID field to meter layer if it doesn't exist
        if "PIPE_ID" not in [f.name() for f in meter_layer.fields()]:
            meter_layer.dataProvider().addAttributes([QgsField("PIPE_ID", QVariant.String)])
            meter_layer.updateFields()
            print("Added PIPE_ID field to meter layer")
        
        # Function to extract nodes (endpoints) from pipe geometry
        def get_pipe_endpoints(pipe_geom):
            if pipe_geom.type() != QgsWkbTypes.LineGeometry:
                return []
            
            points = []
            for part in pipe_geom.asGeometryCollection() if pipe_geom.isMultipart() else [pipe_geom]:
                if part.isMultipart():
                    # Handle multi-line geometries
                    for line in part.asGeometryCollection():
                        if line.type() == QgsWkbTypes.LineGeometry:
                            line_string = line.asPolyline()
                            if line_string:
                                points.append(line_string[0])  # First point
                                points.append(line_string[-1])  # Last point
                else:
                    # Handle single line geometries
                    line_string = part.asPolyline()
                    if line_string:
                        points.append(line_string[0])  # First point
                        points.append(line_string[-1])  # Last point
            
            return points
        
        # Function to check if features are on the same side of a road
        def is_same_side(meter_geom, pipe_geom, meter_point):
            buffer_dist = 5  # Buffer distance in map units, adjust as needed
            
            # First check if the pipe is within maximum acceptable distance
            # Find closest point on the pipe line to the meter
            closest_point_result = pipe_geom.closestSegmentWithContext(meter_point)
            closest_point = closest_point_result[1]
            
            # Calculate distance between meter and closest point on pipe
            distance = meter_point.distance(closest_point)
            
            # If distance exceeds maximum, return False
            if distance > max_distance:
                return False, None, distance
            
            # Get nearby roads using a combined extent instead of unite()
            # Create a new QgsRectangle that includes both meter and pipe
            meter_rect = meter_geom.boundingBox()
            pipe_rect = pipe_geom.boundingBox()
            
            # Manually combine the extents
            min_x = min(meter_rect.xMinimum(), pipe_rect.xMinimum())
            min_y = min(meter_rect.yMinimum(), pipe_rect.yMinimum())
            max_x = max(meter_rect.xMaximum(), pipe_rect.xMaximum())
            max_y = max(meter_rect.yMaximum(), pipe_rect.yMaximum())
            
            # Create a new rectangle with the combined extents
            combined_rect = QgsRectangle(min_x, min_y, max_x, max_y)
            
            # Expand the rectangle by the buffer distance
            combined_rect.grow(buffer_dist * 2)
            
            # Get features within this rectangle
            request = QgsFeatureRequest().setFilterRect(combined_rect)
            
            for road_feat in road_layer.getFeatures(request):
                road_buffer = road_feat.geometry().buffer(buffer_dist, 5)
                # Check if road separates the meter and pipe
                if road_buffer.contains(meter_geom) != road_buffer.contains(pipe_geom):
                    return False, None, distance  # They're on opposite sides of this road
            
            return True, closest_point, distance  # No roads were found to separate them
        
        # Function to find closest node point (either from node layer or pipe endpoints)
        def find_closest_node(meter_point, pipe_geom, pipe_id, node_max_distance):
            best_node = None
            best_distance = float('inf')
            
            # First try nodes from the pipe itself (endpoints)
            if prefer_nodes:
                endpoints = get_pipe_endpoints(pipe_geom)
                for endpoint in endpoints:
                    dist = meter_point.distance(endpoint)
                    if dist < best_distance and dist <= node_max_distance:
                        best_distance = dist
                        best_node = endpoint
            
            # If node layer provided, also check explicit nodes
            if node_index:
                # Find nearby nodes
                nearby_node_ids = node_index.nearestNeighbor(meter_point, 5)
                for nid in nearby_node_ids:
                    node_feat = node_layer.getFeature(nid)
                    
                    # Check if this node belongs to our pipe (if pipe_id field exists in node layer)
                    if "PIPE_ID" in node_feat.fields().names():
                        node_pipe_id = node_feat["PIPE_ID"]
                        if node_pipe_id != pipe_id:
                            continue
                    
                    node_geom = node_feat.geometry()
                    if node_geom and not node_geom.isEmpty():
                        try:
                            node_point = node_geom.asPoint()
                            dist = meter_point.distance(node_point)
                            if dist < best_distance and dist <= node_max_distance:
                                best_distance = dist
                                best_node = node_point
                        except:
                            continue
            
            return best_node, best_distance
        
        # Delete the file if it already exists to avoid conflicts
        if os.path.exists(output_path):
            try:
                os.remove(output_path)
                print(f"Removed existing file: {output_path}")
            except Exception as e:
                print(f"Could not remove existing file: {e}")
                # Try a different filename
                output_path = os.path.join(os.path.dirname(output_path), 
                                           f"spider_output_{int(time.time())}.gpkg")
                print(f"Trying alternative path: {output_path}")
        
        # Create the spider line output file
        spider_fields = QgsFields()
        spider_fields.append(QgsField("CUSTCODE", QVariant.String))
        spider_fields.append(QgsField("PIPE_ID", QVariant.String))
        spider_fields.append(QgsField("DISTANCE", QVariant.Double))
        spider_fields.append(QgsField("CONN_TYPE", QVariant.String))  # Node or Segment
        
        # Create writer using the constructor that works with older QGIS
        try:
            spider_writer = QgsVectorFileWriter(
                output_path,
                "UTF-8",
                spider_fields,
                QgsWkbTypes.LineString,
                meter_layer.crs(),
                "GPKG"
            )
            
            if spider_writer.hasError() != QgsVectorFileWriter.NoError:
                error_message = f"Could not create output file: {spider_writer.errorMessage()}"
                print(error_message)
                iface.messageBar().pushCritical("Error", error_message)
                raise Exception(error_message)
            else:
                print(f"Successfully created output file at {output_path}")
        except Exception as e:
            print(f"Error creating vector file writer: {e}")
            iface.messageBar().pushCritical("Error", f"Could not create output file: {e}")
            raise
        
        # Start editing the meter layer
        match_count = 0
        node_match_count = 0
        segment_match_count = 0
        no_match_count = 0
        
        with edit(meter_layer):
            meter_features = list(meter_layer.getFeatures())
            total_meters = len(meter_features)
            print(f"Processing {total_meters} meters...")
            
            for i, meter in enumerate(meter_features):
                if i % 10 == 0:  # Print progress every 10 features
                    print(f"Processing meter {i+1} of {total_meters}")
                
                meter_geom = meter.geometry()
                
                if not meter_geom or meter_geom.isEmpty():
                    print(f"Skipping meter with invalid geometry")
                    continue
                
                # Try to get a point from the geometry
                try:
                    meter_point = meter_geom.asPoint()
                except:
                    print(f"Could not get point from meter geometry - skipping")
                    continue
                    
                # Get nearest 5 pipes
                nearest_ids = pipe_index.nearestNeighbor(meter_point, 5)
                matched = False
                
                # Keep track of the best match (shortest distance)
                best_match = None
                best_distance = float('inf')
                best_connection_point = None
                best_pipe_feat = None
                best_connection_type = None
                
                for pid in nearest_ids:
                    pipe_feat = pipe_layer.getFeature(pid)
                    pipe_geom = pipe_feat.geometry()
                    pipe_id = pipe_feat["PIPE_ID"] if "PIPE_ID" in pipe_feat.fields().names() else "Unknown"
                    
                    # Check if same side and within max distance
                    same_side, closest_point, distance = is_same_side(meter_geom, pipe_geom, meter_point)
                    
                    if same_side:
                        # Try to find a nearby node first if preferred
                        node_point = None
                        node_distance = float('inf')
                        
                        if prefer_nodes:
                            node_point, node_distance = find_closest_node(
                                meter_point, pipe_geom, pipe_id, node_max_distance
                            )
                        
                        # Determine which connection point to use (node or segment)
                        if node_point and node_distance <= node_max_distance:
                            connection_point = node_point
                            actual_distance = node_distance
                            connection_type = "NODE"
                        else:
                            connection_point = closest_point
                            actual_distance = distance
                            connection_type = "SEGMENT"
                        
                        # Update best match if this is closer
                        if actual_distance < best_distance:
                            best_distance = actual_distance
                            best_connection_point = connection_point
                            best_pipe_feat = pipe_feat
                            best_connection_type = connection_type
                
                # If we found a suitable pipe, create the connection
                if best_pipe_feat is not None:
                    # Update PIPE_ID field in METER
                    meter_id = meter.id()
                    custcode = meter["CUSTCODE"] if "CUSTCODE" in meter.fields().names() else "Unknown"
                    pipe_id = best_pipe_feat["PIPE_ID"] if "PIPE_ID" in best_pipe_feat.fields().names() else "Unknown"
                    
                    meter_layer.changeAttributeValue(
                        meter_id, 
                        meter_layer.fields().lookupField("PIPE_ID"), 
                        pipe_id
                    )
                    
                    # Create spider line connection using the best connection point
                    spider_geom = QgsGeometry.fromPolylineXY([meter_point, best_connection_point])
                    
                    # Create feature for spider line
                    spider_feat = QgsFeature(spider_fields)
                    spider_feat.setGeometry(spider_geom)
                    spider_feat.setAttributes([
                        str(custcode), 
                        str(pipe_id), 
                        float(best_distance), 
                        best_connection_type
                    ])
                    
                    # Add feature to output file
                    spider_writer.addFeature(spider_feat)
                    
                    matched = True
                    match_count += 1
                    
                    # Track match type
                    if best_connection_type == "NODE":
                        node_match_count += 1
                    else:
                        segment_match_count += 1
                        
                if not matched:
                    custcode = meter["CUSTCODE"] if "CUSTCODE" in meter.fields().names() else "Unknown"
                    print(f"No matching pipe found on the same side of the road for Meter {custcode}")
                    no_match_count += 1
        
        # Clean up and finalize
        del spider_writer
                
        # Show results
        message = (f"Matched {match_count} meters to pipes and created spider lines. "
                  f"({node_match_count} to nodes, {segment_match_count} to segments). "
                  f"{no_match_count} meters could not be matched.")
        print(message)
        iface.messageBar().pushSuccess("Success", message)
                
        # Refresh canvas to show changes
        iface.mapCanvas().refreshAllLayers()
                
        # Add the new spider line layer to the map
        spider_layer = iface.addVectorLayer(output_path, "Spider Lines", "ogr")
        if not spider_layer or not spider_layer.isValid():
            iface.messageBar().pushWarning("Warning", "Spider line layer could not be added to the map")
            print("Spider line layer could not be added to the map")
        else:
            print("Spider line layer added to map")
            
            # Create a basic symbology to differentiate between node and segment connections
            categories = []
            
            # Create a symbol for node connections
            node_symbol = QgsSymbol.defaultSymbol(QgsWkbTypes.LineGeometry)
            node_symbol.setColor(QColor(0, 255, 0))  # Green for node connections
            node_symbol.setWidth(0.5)
            node_category = QgsRendererCategory("NODE", node_symbol, "Connected to Node")
            categories.append(node_category)
            
            # Create a symbol for segment connections
            segment_symbol = QgsSymbol.defaultSymbol(QgsWkbTypes.LineGeometry)
            segment_symbol.setColor(QColor(255, 0, 0))  # Red for segment connections
            segment_symbol.setWidth(0.5)
            segment_category = QgsRendererCategory("SEGMENT", segment_symbol, "Connected to Segment")
            categories.append(segment_category)
            
            # Create and set the renderer
            renderer = QgsCategorizedSymbolRenderer("CONN_TYPE", categories)
            spider_layer.setRenderer(renderer)
            
            # Refresh to show the new symbology
            spider_layer.triggerRepaint()
            iface.mapCanvas().refresh()

# Create and show the dialog
dialog = LayerSelectionDialog()
dialog.show()
