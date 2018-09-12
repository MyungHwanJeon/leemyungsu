import os
import rospkg
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QIcon, QPixmap, QImage, QPainter, QPen, QFontMetrics
from python_qt_binding.QtWidgets import QApplication, QWidget, QLabel, QSlider, QCheckBox, QVBoxLayout, QSizePolicy
        
class SliderWithValue(QSlider, QtCore.QObject):
    slidervalue = QtCore.Signal(int)
    def __init__(self, x, parent=None):
        QtCore.QObject.__init__(self)
        self._x = x
        super(SliderWithValue, self).__init__(parent)
        self.stylesheet = """
        QSlider::groove:horizontal {
                background-color: #222;
                height: 20px;
        }
        QSlider::handle:horizontal {
            border: 1px #438f99;
            border-style: outset;
            margin: -2px 0;
            width: 10px;
            height: 20px;
            background-color: #438f99;
        }
        QSlider::sub-page:horizontal {
            background: #4B4B4B;
        }
        """
        self.setStyleSheet(self.stylesheet)
    @property
    def x(self):
    	return self._x
    @x.setter
    def x(self, new_x):
    	self._x = new_x
        self.slidervalue.emit(new_x)
    
    def paintEvent(self, event):
        QSlider.paintEvent(self, event)

        curr_value = str(self.value())
        round_value = round(float(curr_value), 4)

        painter = QPainter(self)
        painter.setPen(QPen(QtCore.Qt.white))

        font_metrics = QFontMetrics(self.font())
        font_width = font_metrics.boundingRect(str(round_value)).width()
        font_height = font_metrics.boundingRect(str(round_value)).height()

        rect = self.geometry()
        if self.orientation() == QtCore.Qt.Horizontal:
            horizontal_x_pos = rect.width() - font_width - 5
            horizontal_y_pos = rect.height() * 0.75

            painter.drawText(QtCore.QPoint(horizontal_x_pos, horizontal_y_pos), str(round_value))
            self.x=self.value()
        else:
            pass
        painter.drawRect(rect)
        
class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        Black = np.zeros((300,400,3), np.uint8)
        self.is_decay_auto = 0
        self.is_frame_auto = 0
        self.is_segsz_auto = 0
        self.bridge = CvBridge()
        self.d_image_sub = rospy.Subscriber("/fome/direction",Image,self.fome_dir)
        self.m_image_sub = rospy.Subscriber("/fome/magnitude",Image,self.fome_mag)
        self.c_image_sub = rospy.Subscriber("/fome/cluster",Image,self.fome_cls)
#        self.r_image_sub = rospy.Subscriber("/dvs_rendering",Image,self.fome_raw)
        self.r_image_sub = rospy.Subscriber("/dvs/image_raw",Image,self.fome_raw)
        self.decay_pub = rospy.Publisher("/fome/decay_rate",String,queue_size=1)
        self.frame_pub = rospy.Publisher("/fome/frame_rate",String,queue_size=1)
        self.segsz_pub = rospy.Publisher("/fome/segment_size",String,queue_size=1)
        self.setObjectName('fome_gui')
        
        rp = rospkg.RosPack()
        self._widget = QWidget()
        ui_file = os.path.join(rp.get_path('fome_gui'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('fome_gui')
        
    	height, width, byteValue = Black.shape
        byteValue = byteValue * width
        self.d_image = QImage(Black, width, height, byteValue, QImage.Format_RGB888)
        self.m_image = QImage(Black, width, height, byteValue, QImage.Format_RGB888)
        self.r_image = QImage(Black, width, height, byteValue, QImage.Format_RGB888)
        self.c_image = QImage(Black, width, height, byteValue, QImage.Format_RGB888)
        
        self.d_label = QLabel(self._widget.graphicsView_1)
        d_image = QPixmap.fromImage(self.d_image)
        self.d_label.setPixmap(d_image)
        
        self.m_label = QLabel(self._widget.graphicsView_2)
        m_image = QPixmap.fromImage(self.m_image)
        self.m_label.setPixmap(m_image)
        
        self.r_label = QLabel(self._widget.graphicsView_3)
        r_image = QPixmap.fromImage(self.r_image)
        self.r_label.setPixmap(r_image)
            
        self.c_label = QLabel(self._widget.graphicsView_4)
        c_image = QPixmap.fromImage(self.c_image)
        self.c_label.setPixmap(c_image)
                
        @QtCore.Slot(int)
        def decay_moved(r):
        	if self.is_decay_auto == 0:
        		self.decay_pub.publish(str(r))
        self.decay_rate = SliderWithValue(QtCore.Qt.Horizontal, 1)
        self.decay_rate.slidervalue.connect(decay_moved)
        self.decay_rate.setMinimum(-20)
        self.decay_rate.setMaximum(30)
        self.decay_rate.setTickInterval(1)
        self.decay_rate.setSingleStep(1)
        self.decay_rate.setPageStep(1)
        self.decay_rate.setTickPosition(QSlider.TicksBelow)
        self.decay_rate.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.decay_rate.setValue(-10)
        self.decay_pub.publish(str(self.decay_rate.slidervalue))
        self._widget.decay_rate.addWidget(self.decay_rate)
        
        @QtCore.Slot(int)
        def decay_auto(args):
        	self.is_decay_auto = args
        	if args == 2:
        		self.decay_pub.publish("auto")
        		self.decay_pub.unregister
        		del self.decay_pub
        	if args == 0:
        		self.decay_pub = rospy.Publisher("/fome/decay_rate",String,queue_size=1)
        self._widget.decay_auto.stateChanged.connect(decay_auto)


        
        @QtCore.Slot(int)
        def frame_moved(r):
        	if self.is_frame_auto == 0:
        		self.frame_pub.publish(str(r))
        self.frame_rate = SliderWithValue(QtCore.Qt.Horizontal, 1)
        self.frame_rate.slidervalue.connect(frame_moved)
        self.frame_rate.setMinimum(10)
        self.frame_rate.setMaximum(2000)
        self.frame_rate.setTickInterval(1)
        self.frame_rate.setSingleStep(1)
        self.frame_rate.setPageStep(1)
        self.frame_rate.setTickPosition(QSlider.TicksBelow)
        self.frame_rate.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.frame_rate.setValue(100)
        self._widget.frame_rate.addWidget(self.frame_rate)
        
        @QtCore.Slot(int)
        def frame_auto(args):
        	self.is_frame_auto = args
        	if args == 2:
        		self.frame_pub.publish("auto")
        		self.frame_pub.unregister
        		del self.frame_pub
        	if args == 0:
        		self.frame_pub = rospy.Publisher("/fome/frame_rate",String,queue_size=1)
        self._widget.frame_auto.stateChanged.connect(frame_auto)



        @QtCore.Slot(int)
        def segsz_moved(r):
        	if self.is_segsz_auto == 0:
        		self.segsz_pub.publish(str(r))
        self.segsz_rate = SliderWithValue(QtCore.Qt.Horizontal, 1)
        self.segsz_rate.slidervalue.connect(segsz_moved)
        self.segsz_rate.setMinimum(100)
        self.segsz_rate.setMaximum(10000)
        self.segsz_rate.setTickInterval(10)
        self.segsz_rate.setSingleStep(10)
        self.segsz_rate.setPageStep(10)
        self.segsz_rate.setTickPosition(QSlider.TicksBelow)
        self.segsz_rate.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.segsz_rate.setValue(4320)
        self._widget.segsz_rate.addWidget(self.segsz_rate)
        
        @QtCore.Slot(int)
        def segsz_auto(args):
        	self.is_segsz_auto = args
        	if args == 2:
        		self.segsz_pub.publish("auto")
        		self.segsz_pub.unregister
        		del self.segsz_pub
        	if args == 0:
        		self.segsz_pub = rospy.Publisher("/fome/segsz_rate",String,queue_size=1)
        self._widget.segsz_auto.stateChanged.connect(segsz_auto)
        
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle('Estimated direction from events')
        context.add_widget(self._widget)
        
    def fome_dir(self,data):
    	try:
    		cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	except CvBridgeError as e:
    		print(e)
    	height, width, byteValue = cvImage.shape
        byteValue = byteValue * width
        cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB, cvImage)
    	d_image = QImage(cvImage, width, height, byteValue, QImage.Format_RGB888)
        d_image = QPixmap.fromImage(d_image)
        d_image = d_image.scaledToHeight(300)
        self.d_label.setPixmap(d_image)
        
    def fome_mag(self,data):
    	try:
    		cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	except CvBridgeError as e:
    		print(e)
    	height, width, byteValue = cvImage.shape
        byteValue = byteValue * width
        cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB, cvImage)
    	m_image = QImage(cvImage, width, height, byteValue, QImage.Format_RGB888)
        m_image = QPixmap.fromImage(m_image)
        m_image = m_image.scaledToHeight(300)
        self.m_label.setPixmap(m_image)
        
    def fome_cls(self,data):
		try:
			cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		height, width, byteValue = cvImage.shape
		byteValue = byteValue * width
		cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB, cvImage)
		c_image = QImage(cvImage, width, height, byteValue, QImage.Format_RGB888)
		c_image = QPixmap.fromImage(c_image)
		c_image = c_image.scaledToHeight(300)
		self.c_label.setPixmap(c_image)
    	
    def fome_raw(self,data):
    	try:
    		cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	except CvBridgeError as e:
    		print(e)
    	height, width, byteValue = cvImage.shape
        byteValue = byteValue * width
        cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB, cvImage)
        r_image = QImage(cvImage, width, height, byteValue, QImage.Format_RGB888)
        r_image = QPixmap.fromImage(r_image)
        r_image = r_image.scaledToHeight(300)
        self.r_label.setPixmap(r_image)
    	    		
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
