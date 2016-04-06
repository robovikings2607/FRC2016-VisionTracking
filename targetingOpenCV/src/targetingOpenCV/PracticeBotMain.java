package targetingOpenCV;


import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.InputStream;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.FrameGrabber;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.grip.core.sources.IPCameraFrameGrabber;

public class PracticeBotMain implements MouseListener {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	
	private BufferedImage imgBuf;
	private JFrame binFrame, imgFrame, aimFrame;
	private JPanel aimPanel;
	private ImageIcon binIcon, imgIcon;
    private ArrayList<MatOfPoint2f> targets;
    private ArrayList<Rect> targetBoundingRects; 
    private ArrayList<RotatedRect> targetRotatedRects;
    private ArrayList<Double> targetHeights;
    private int selectedTargetIndex;
    private DecimalFormat floatFmt, intFmt;
    private Calibrate calib;
	private ArrayList<MatOfPoint> contours;
	private ArrayList<Point> hullPoints;
	private Font normalFont = new Font("Arial", Font.PLAIN, 10),
				 boldFont = new Font("Arial", Font.BOLD, 12);
    //private final double cameraVertFoVRad = Math.atan(99.25 / 208.5) / 1.212;  // measured
    	// the above is tuned to get expected distance from the sample image & measurements
    	//	but an uncalibrated sample image
    	// with proper measurements the denominator should be 2.0.  Think the sample image
    	// is poor (more than the wall is visible, so vertical scale is inaccurate) - on top of the
    	// fact that the camera isn't calibrated yet
//    private final double cameraVertFoVRad = Math.atan(17.25 / 32) / 1.4;  // tuned
	  private final double cameraVertFoVRad = Math.toRadians(25.5);
	// note per M1013 documentation, horizontal FoV is 67deg, vertical is 51deg, 2.8mm focalLength, 6.35mm sensor
//    private final double cameraVertFoVRad = Math.toRadians(51.0);
    private final double targetHeightInches = 12.0;  	
    
    private final double targetHeightOffFloorInches = 85.0;			//89.0;   
    private final double armPivotHeightInches = 12.0;
    
    private final double cameraMountHeightInches = 12.75;
    private final double cameraMountAngleDeg = 53.0;		//52.0; //55.5;
//    private final double cameraMountedMidFoVAngle = 180 - cameraMountAngleDeg;
    
    private boolean saveFrames = false, pausePlayback = false, needFramePath = true;
    private String savedFramePath;
    private int savedFrameCount;
    private double degToRotate, targetAngleInFOV;
    
    private class imageSaver extends Thread {
        
        private BufferedImage imageToSave;
        private int imageCount;
        private String imageInfo;
        
        public imageSaver(BufferedImage image, String imageType, int count) {
            imageToSave = image;
            imageCount = count;
            imageInfo = imageType;
        }
        
        public void run() {
            StringBuffer sb = new StringBuffer(savedFramePath);
            sb.append(File.separatorChar).append(imageInfo).append(".");
            sb.append(imageCount).append(".jpg");
            try {
                ImageIO.write(imageToSave, "jpg", new File(sb.toString()));
            } catch (Exception e) {}
        }
    }
    
    private class savedImageStreamer {
        private Mat img;
        private ArrayList<Path> il1 = new ArrayList(), 
                                il2 = new ArrayList(),
                                il3 = new ArrayList(),
                                il4 = new ArrayList();
        private Iterator<Path> it1, it2, it3, it4;
        private String fileName;
        
        public Mat grab() {
        	if (it1.hasNext()) {
        		fileName = it1.next().toString();	
        	} else {
        		if (it2.hasNext()) {
        			fileName = it2.next().toString();
        		} else {
        			if (it3.hasNext()) {
        				fileName = it3.next().toString();
        			} else {
        				if (it4.hasNext()) {
        					fileName = it4.next().toString();
        				} else {
        					return null;
        				}
        			}
        		}
        	}
        	return Imgcodecs.imread(fileName);
        }
        
        public String getFileName() {
        	System.out.println(fileName);
        	return fileName;
        }
        
        public savedImageStreamer(String savedImageDir) {
            try {           
                for (Path p : Files.newDirectoryStream(Paths.get(savedImageDir), "Camera.?.jpg"))
                    il1.add(p);
                for (Path p : Files.newDirectoryStream(Paths.get(savedImageDir), "Camera.??.jpg"))
                    il2.add(p);
                for (Path p : Files.newDirectoryStream(Paths.get(savedImageDir), "Camera.???.jpg"))
                    il3.add(p);
                for (Path p : Files.newDirectoryStream(Paths.get(savedImageDir), "Camera.????.jpg"))
                    il4.add(p);
                Collections.sort(il1);
                Collections.sort(il2);
                Collections.sort(il3);
                Collections.sort(il4);
                it1 = il1.iterator();
                it2 = il2.iterator();
                it3 = il3.iterator();
                it4 = il4.iterator();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    
    
    
	public PracticeBotMain() {
		binFrame = new JFrame("Binary Image");
		imgFrame = new JFrame("Camera Image");
		aimFrame = new JFrame("Side view");
		binIcon = new ImageIcon();
		imgIcon = new ImageIcon();
        binFrame.getContentPane().add(new JLabel(binIcon));
        imgFrame.getContentPane().add(new JLabel(imgIcon));
        binFrame.pack();
        binFrame.setVisible(true);
        imgFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        imgFrame.pack();
        imgFrame.setVisible(true);

        floatFmt = new DecimalFormat("#.00");
        intFmt = new DecimalFormat("###");
        
//        calib = new Calibrate();
//        calib.performCalibration();
        
        targetBoundingRects = new ArrayList<Rect>();	// the bounding box of each target
        targets = new ArrayList<MatOfPoint2f>();			// the points of each convex hull describing the target
        targetRotatedRects =  new ArrayList<RotatedRect>(); // the bounding Rotated Rect of each target
        targetHeights = new ArrayList<Double>();			// the height in pixels of each target (actually height of convex hull side)
        contours = new ArrayList<MatOfPoint>();
        hullPoints = new ArrayList<Point>();
        Robot.setUseMDNS(true);
        Robot.setTeam(2607);
	}

	private void setupButtons() {
		Calendar cal = Calendar.getInstance();
		JFrame btnFrame = new JFrame("Control Panel");
        btnFrame.setLayout(new FlowLayout());
        btnFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        JButton btnSaveFrames = new JButton("Save Frames");
        btnSaveFrames.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent ev) {
                if (!saveFrames) {
                    ((JButton)ev.getSource()).setText("Stop Saving");
                    savedFramePath = "savedImages-" + cal.get(Calendar.YEAR) + "-" + (cal.get(Calendar.MONTH) 
                    		+ "-" + cal.get(Calendar.DAY_OF_MONTH) + "." + cal.get(Calendar.HOUR_OF_DAY)
                    		+ "-" + cal.get(Calendar.MINUTE) + "-" + cal.get(Calendar.SECOND));
                    new File(savedFramePath).mkdir();
                    saveFrames = true;
                    needFramePath = false;
                } else {
                    ((JButton)ev.getSource()).setText("Save Frames");
                    saveFrames = false;
                    needFramePath = true;
                }
            }
        }); 
        btnFrame.add(btnSaveFrames);
        btnFrame.pack();
        btnFrame.setVisible(true);
	}
	
	
	private Mat loadFromFile(String fileName) {
	
		Mat origImg = Imgcodecs.imread(fileName);
		if (origImg != null) {
			System.out.println("Image is (" + origImg.cols() + "x" + origImg.rows() + ")");
/*
			Mat correctedImg = new Mat();
			Imgproc.undistort(origImg, correctedImg, calib.getCameraMatrix(), calib.getDistortMatrix());
			binFrame.setSize(correctedImg.width(), correctedImg.height());
			imgFrame.setSize(correctedImg.width(), correctedImg.height());
			imgBuf = matToBuf(correctedImg);
			return correctedImg;
*/
			binFrame.setSize(origImg.width(), origImg.height());
			imgFrame.setSize(origImg.width(), origImg.height());
			imgBuf = matToBuf(origImg);
			return origImg;

		} else {
			System.out.println("couldn't load image");
			return null;
		}
	}
	
	private Mat binarizeHSV(Mat origImg) {
		Mat hsvImg = new Mat();
		Imgproc.cvtColor(origImg, hsvImg, Imgproc.COLOR_BGR2HSV);
		Mat binImg = new Mat(hsvImg.size(), CvType.CV_8UC1);

		Core.inRange(hsvImg, new Scalar(24,0,73), new Scalar(94,255,255), binImg);
		Imgproc.morphologyEx(binImg, binImg, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6)));
		return binImg;
	}
	
	private Mat binarizeSubt(Mat origImg) {
		// extract the green and red channels;  subtract red from green
		// and threshold the result
		Mat greenChannel = new Mat();
		Mat redChannel = new Mat();
		
		Mat diff = new Mat();
		Mat binImg = new Mat(origImg.size(), CvType.CV_8UC1);

		Core.extractChannel(origImg, greenChannel, 1);
		Core.extractChannel(origImg, redChannel, 2);

		Core.subtract(greenChannel, redChannel, diff);
		
		Imgproc.threshold(diff, binImg, 0, 255, Imgproc.THRESH_OTSU);
		return binImg;
	}

	private void findTargets(Mat binImg) {


        contours.clear();
        targetBoundingRects.clear();
        targets.clear();
        targetRotatedRects.clear();        
        targetHeights.clear();
        
		Imgproc.findContours(binImg, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        System.out.println("Found " + contours.size() + " contours");
           
        double maxRatio = 0.0;
        selectedTargetIndex = -1;
        for (MatOfPoint m : contours) {
        	MatOfInt hull = new MatOfInt();
        	hullPoints.clear();
        	Imgproc.convexHull(m, hull, true);
        	// the hull is actually the clockwise array of contour points it's comprised of...
        	// "clockwise" means it starts from the bottom left of the contour, and moves clockwise from there
        	//	- last param false would go counter-clockwise, i.e. start from bottom right
        	// store this as a MatOfPoint2f for easy use
        	
        	List<Point> contourPoints = m.toList();        	
        	for (Integer i : hull.toList()) {
        		hullPoints.add(contourPoints.get(i));
        	}
        	MatOfPoint2f matHull= new MatOfPoint2f();
        	matHull.fromList(hullPoints);
        	RotatedRect e = Imgproc.minAreaRect(matHull);
        	Rect rect = e.boundingRect();
        	double ratio = (double)rect.width / (double)rect.height;
//			System.out.println("bounding rect area: " + rect.area() + " aspect ratio: " + 
//					ratio);

			//if (rect.width > 40 && rect.width < 1000 && rect.height > 40 && rect.height < 1000) {
			// actual target ratio is 20in x 12in = 1.6
			
			// for now this still selects based on bounding box area and ratio
			// for more precision, use hull:
			//		identify nearly rectangular hulls, find their corners, get center point
        	
			if (rect.area() >= 1000 && rect.area() <= 9000 && ratio >= 1.4 && ratio <= 3.3 //&& rect.y < binImg.size().height * .53
					) {
				System.out.println("Rect area: " + rect.area());
//				System.out.println("\tadding target");
				targetBoundingRects.add(rect);
				targets.add(matHull);
				targetRotatedRects.add(e);
				Point[] hp = matHull.toArray();
				double[] sideHeights = new double[2]; 
				int side = 0;
				for (int i = 0; i < hp.length - 1; i++) {
					Point nextPoint = hp[i+1];
					double dy = nextPoint.y - hp[i].y;
					double slope = Math.abs(dy / (nextPoint.x - hp[i].x));
					if (slope >= 1.0) {
						sideHeights[side] += Math.abs(dy);
						//g.drawLine((int)hp[i].x, (int)hp[i].y, (int)nextPoint.x, (int)nextPoint.y);
					} else {
						if (++side > 1) side = 1;
					}
				}
				targetHeights.add(Math.max(sideHeights[0], sideHeights[1]));
				if (ratio > maxRatio) { 
					selectedTargetIndex = targetBoundingRects.size() - 1;
					maxRatio = ratio;
				}
				// may also want to save the raw contour?
			}
        }        
        
	}

	// returns distance to target in whatever unit the targetHeightWorld is specified in
	// (e.g. inches, feet, centimeters, etc)
	
	private double targetDistance(Rect boundingRect, RotatedRect rotatedRect, double targetHeightPixels, double imgHeightPixels, 
									double imgWidthPixels, double targetHeightWorld) {
		// M1011 horizFOV 47, verticalFOV 43.5
		// M1013 horizFOV 67, verticalFOV 51
		
		// camera mount is rotated 90deg, so horizFOV is the y axis and vertiFOV is the x axis in image
		
/*				
		System.out.println("targetHeightPixels: " + targetHeightPixels);
		System.out.println("boundingRect.height: " + boundingRect.height);
		
		double x = boundingRect.x + (boundingRect.width / 2);
		double y = boundingRect.y + (boundingRect.height / 2);
		System.out.println("boundingRect center: (" + x + "," + y + ")");
		y = -((2 * (y / imgHeightPixels)) - 1);
		System.out.println("y: " + y);
		
		double range = (targetHeightOffFloorInches - cameraMountHeightInches) / Math.tan((y * 67.0/2.0 + cameraMountAngleDeg)*Math.PI/180.0);
		System.out.println("Range (Ross): " + range);
*/		
// M1013		double targetAngleInFOV = 67 - (((boundingRect.y + ((boundingRect.height) / 2))/imgHeightPixels) * 67);
		targetAngleInFOV = 47 - (((boundingRect.y + ((boundingRect.height) / 2))/imgHeightPixels) * 47);
//		double azimuth = this.boundAngle0to360Degrees(x*kHorizontalFOVDeg/2.0 + heading - kShooterOffsetDeg);

		/*
		 * imageWidthPixels		x distance from shot (<0 = left)
		 * ---------------- =   ------------------------
		 * 51deg				degrees to rotate (opposite sign)
		 *
		 * shot center is 187 from HH image 1263, 173 from HH image 529
		 * 187 looks a little left; 173 looks pretty centered.  Try 175 to start  
		 *
		 */
		
		double x = boundingRect.x + (boundingRect.width / 2);  
		System.out.println("x of target center from boundingRect: " + x);
		System.out.println("x of target center from rotatedRect: " + rotatedRect.center.x);
//		M1013 double degToRotate = ((x - 175) * 51.0) / imgWidthPixels;
		degToRotate = ((x - 175) * 43.5) / imgWidthPixels;		// M1011
		System.out.println("degToRotate: " + degToRotate);
/*		System.out.println("Target Angle in FOV: " + targetAngleInFOV);
		double range2 = (89 - 12.75) / (Math.tan(targetAngleInFOV - (Math.PI/2 - (Math.PI - Math.toRadians(51) - 1.169372/2))));
		System.out.println("Range (Griff): " + range2);
		
		double fovWorld = targetHeightWorld * (imgHeightPixels / boundingRect.height);		
		double distToTarget = (fovWorld / 2.0) / Math.tan(Math.toRadians(33.5 + cameraMountAngleDeg));
		System.out.println("distToTarget: " + distToTarget);
		*/
		Robot.getTable().putNumber("targetAngleInFOV", targetAngleInFOV);
		Robot.getTable().putNumber("degToRotate", degToRotate);
		return targetAngleInFOV;
		
		
		// the target height off the floor is known (bottom is 85in, bottom of reflective tape is 83in)
		// double range = (kTopTargetHeightIn-kCameraHeightIn)/Math.tan((y*kVerticalFOVDeg/2.0 + kCameraPitchDeg)*Math.PI/180.0);
			// y is center of the bounding rect, which is 89in off the ground
			// center of goal opening is 97in off ground
			// (89in - 12.75in) / tan((y * 67.0/2.0 + cameraMountAngleDeg)*Math.PI/180.0) 

		
		// TargetHeightOffFloor - CameraMountHeight  
		// ----------------------------------------
		// tan (targetAngleOffGournd - (90 - (180 - cameraMountAngle - cameraVertFoV/2.0)))
		
		// targetAngleOffGround: calc proportionally, top is FoV, bottom is 0
		//
		
		// note per M1013 documentation, horizontal FoV is 67deg, vertical is 51deg, 2.8mm focalLength, 6.35mm sensor
					
	}
	
	private double aimAngleDeg(Rect target, double distanceToTarget) {
		// we know the target height in the world coordinate system
		//		and
		// we know the mounted height of the arm
		// the difference between these two is the opposite side length
		// the previously calculated distance is the adjacent side length
		// angle to aim is therefore atan(opposite/adjacent)
		
		// may need to add a little to the target height to aim into the target as
		// opposed to just at the bottom...anyway, tune to get consistent results
		
		double opposite = targetHeightOffFloorInches - armPivotHeightInches;
		System.out.println("aimAngle without Griffin's table: " + Math.toDegrees(Math.atan(opposite / distanceToTarget)));
		
		// per Griffin's table:
		double aimRad = Math.atan((distanceToTarget / (targetHeightOffFloorInches - cameraMountHeightInches)));
		aimRad -= Math.PI / 2 - (Math.PI - Math.toRadians(cameraMountAngleDeg) - 1.16937/2.0);
		double aimAngleDeg = 90 - Math.toDegrees(aimRad);
		//Robot.getTable().putNumber("aimAngleDeg", aimAngleDeg);
		return aimAngleDeg;
	}
	
	private BufferedImage drawTargets(BufferedImage img) {

		
		if (selectedTargetIndex >= 0) {
			Graphics2D g = img.createGraphics();
			g.setFont(normalFont);
			g.setColor(Color.GREEN);
			Rect r = targetBoundingRects.get(selectedTargetIndex);
			RotatedRect rr = targetRotatedRects.get(selectedTargetIndex);
			g.drawRect(r.x, r.y, r.width, r.height);
//			g.drawOval((int)rr.center.x, (int)rr.center.y, (int)rr.size.width, (int)rr.size.height);
			System.out.println("ratio: " + ((double)r.width / (double)r.height) + " area: " + r.area());
			g.drawOval(r.x + (r.width / 2), r.y + (r.height / 2) - 5, 2, 2);
			double d = targetDistance(r, rr, targetHeights.get(selectedTargetIndex), img.getHeight(), img.getWidth(), targetHeightInches);
			
			//System.out.println("distance: " + d);
			//System.out.println("rect: (" + r.x + ',' + r.y + ") height: " + r.height + " width: " + r.width);
			g.drawString(floatFmt.format(d), r.x, r.y + r.height + 10);
			int startCol = r.x + r.width + 2;
			g.setFont(boldFont);
			g.drawString("degToRotate: " + floatFmt.format(degToRotate), startCol, r.y);
			g.drawString("SP: " + floatFmt.format(Robot.getTable().getNumber("robotTurnSP", 999)), startCol, r.y + boldFont.getSize());
			g.drawString("PV: " + floatFmt.format(Robot.getTable().getNumber("robotTurnPV", 999)), startCol, r.y + (boldFont.getSize() * 2));
			g.drawString("Info: " + Robot.getTable().getString("robotInfo", "n/a"), startCol, r.y + (boldFont.getSize() * 3));

		} else {
			Robot.getTable().putNumber("targetAngleInFOV", -999);
			Robot.getTable().putNumber("degToRotate", -999);
		}
/*	
		for (int x = 0; x < targetBoundingRects.size(); x++) {
			if (x == selectedTargetIndex) g.setColor(Color.GREEN);
			else g.setColor(Color.RED);
			Rect r = targetBoundingRects.get(x);
			g.drawRect(r.x, r.y, r.width, r.height);
			g.drawOval(r.x + (r.width / 2), r.y + (r.height / 2) - 5, 2, 2);
			double d = targetDistance(r, targetHeights.get(x), img.getHeight(), targetHeightInches);
			System.out.println("distance: " + d);
			System.out.println("rect: (" + r.x + ',' + r.y + ") height: " + r.height + " width: " + r.width);
			g.drawString(floatFmt.format(d), r.x, r.y + r.height + 10);
			g.setColor(Color.PINK);
			
			//double th = aimAngleDeg(r, d);
			//System.out.println("aim angle: " + th);		
		}
*/
/*		
		for (int x = 0; x < targetRotatedRects.size(); x++) {
			int x1 = (int)targets.get(x).toList().get(0).x;
			int y1 = (int)targets.get(x).toList().get(0).y;
			int x2 = (int)targets.get(x).toList().get(1).x;
			int y2 = (int)targets.get(x).toList().get(1).y;
			g.drawLine(x1, y1, x1, y1);
			g.drawLine(x2, y2, x2, y2);
			x1 = x2;
			y1 = y2;
			x2 = (int)targets.get(x).toList().get(2).x;
			y2 = (int)targets.get(x).toList().get(2).y;

		}
*/
		return img;

	}
	
	private BufferedImage drawGreenZone(BufferedImage img) {
		
		   ColorModel cm = img.getColorModel();
		    boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
		    WritableRaster raster = img.copyData(img.getRaster().createCompatibleWritableRaster());
		    
		BufferedImage gz = new BufferedImage(cm, raster, isAlphaPremultiplied, null);
		Graphics2D g = gz.createGraphics();
		
		g.setColor(Color.GREEN);
		g.setStroke(new BasicStroke(4));

		// draw side angle green zone
		// target from side when aligned 	  		rect: (132,319) height: 52 width: 87
		g.drawRect(138, 284, 78, 78);

		// draw outerworks center green zone
		// target from outer works when aligned:    rect: (149,346) height: 45 width: 78
		g.setColor(Color.YELLOW);
		g.drawRect(144, 304, 74, 75);				// 74, 75
		return gz;

		/*	original green zone, from image that was smaller
		g.drawRect(180, 297, 80, 88);
		g.drawLine(220, 297, 220, 385);
		g.drawLine(180, 341, 260, 341);
		 */

	
	}
	
	private BufferedImage drawTargets(Mat origImg) {
		return drawTargets(matToBuf(origImg));
	}
	
    private BufferedImage matToBuf(Mat m) {
        MatOfByte mByte = new MatOfByte();
        Imgcodecs.imencode(".jpg", m, mByte);
        try {
            return ImageIO.read(new ByteArrayInputStream(mByte.toArray()));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }        
    }
    
    private Mat bufToMat(BufferedImage image) {
        byte[] data = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
        Mat mat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC3);
        mat.put(0, 0, data);
        return mat;
    }
    
    public Mat loadFromJavaCV(String url) {
    	    	
        IPCameraFrameGrabber camGrabber = null;
        
        System.out.println("opening stream...");
        
        try {
            camGrabber = new IPCameraFrameGrabber(url, 3000, 2000, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            System.out.printf("IPCameraFrameGrabber ctor: %s\n", e.getMessage());
        }
        
        boolean keepTrying = true;
        while (keepTrying) {
            String stepName = "null";
            try {
                stepName = "camGrabber.start()";
                camGrabber.start();
                stepName = "camGrabber.grab()";
                camGrabber.grab();
                keepTrying = false;
            } catch (Exception e) {
                System.out.printf("%s: %s ... retrying... \n", stepName, e.getMessage());               
                try { Thread.sleep(1500); } catch (Exception e1) {}
            }
        }        
        
        try {
            System.out.println("starting capture");
            BufferedImage img = camGrabber.grabBufferedImage();
//            imgFrame.setSize(img.getWidth(), img.getHeight());
//            binFrame.setSize(img.getWidth(), img.getHeight());
//            Mat camMat = calib.getCameraMatrix();
//            Mat distort = calib.getDistortMatrix();
//            Mat correctedImg = new Mat();
            do {
//            	Imgproc.undistort(bufToMat(img), correctedImg, camMat, distort);
            	Mat t = bufToMat(img), tt = new Mat();
            	Core.flip(t.t(), tt, 1);
            	imgFrame.setSize(tt.cols(), tt.rows());
            	binFrame.setSize(tt.cols(), tt.rows());
            	Mat b = binarizeSubt(tt);
        		binIcon.setImage(matToBuf(b));
        		findTargets(b);
            	BufferedImage gz = drawGreenZone(drawTargets(matToBuf(tt)));
        		imgIcon.setImage(gz);
        		imgFrame.repaint();
        		if (saveFrames || Robot.getTable().getBoolean("robotShooting", false)) {
        			if (needFramePath) {
        				Calendar cal = Calendar.getInstance();
        				savedFramePath = "savedImages-" + cal.get(Calendar.YEAR) + "-" + (cal.get(Calendar.MONTH) 
        						+ "-" + cal.get(Calendar.DAY_OF_MONTH) + "." + cal.get(Calendar.HOUR_OF_DAY)
        						+ "-" + cal.get(Calendar.MINUTE) + "-" + cal.get(Calendar.SECOND));
        				new File(savedFramePath).mkdir();
        				needFramePath = false;
        			}

        			new imageSaver(img, "Camera", savedFrameCount).start();
        			new imageSaver(gz, "GreenZone", savedFrameCount).start();
        			savedFrameCount += 1;
        		}
        		binFrame.repaint();
        		img = camGrabber.grabBufferedImage();
            } while(true);
        } catch (Exception e) {
            System.out.printf("camGrabber.grab(): %s\n", e.getMessage());
            try { Thread.sleep(1500); } catch (Exception e1) {}
        } 
            
        try {
            camGrabber.stop();
            camGrabber.release();
        } catch (Exception e) {
            
        }
        
        return null;
    }
    
    public BufferedImage drawFilename(BufferedImage img) {
		   ColorModel cm = img.getColorModel();
		    boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
		    WritableRaster raster = img.copyData(img.getRaster().createCompatibleWritableRaster());
		    
		BufferedImage gz = new BufferedImage(cm, raster, isAlphaPremultiplied, null);
		Graphics2D g = gz.createGraphics();
		g.setFont(new Font("Arial", Font.PLAIN, 10));
		
		return gz;
    }
    		
    public void process(savedImageStreamer strm) {
    	
    	Mat orig;
    	orig = strm.grab();
		imgFrame.setSize(orig.width(), orig.height());
		binFrame.setSize(orig.width(), orig.height());
		imgFrame.addMouseListener(this);
			
    	while (orig != null) {
    		System.out.println("fileName: " + strm.getFileName());
    		Mat b = binarizeSubt(orig);
    		binIcon.setImage(matToBuf(b));
    		findTargets(b);
    		imgIcon.setImage(drawGreenZone(drawTargets(orig)));
    		imgFrame.repaint();
    		binFrame.repaint();
    		if (!pausePlayback) orig = strm.grab();
    		try {Thread.sleep(20);} catch (Exception e) {}
    	}
    	
    }
    
    // if called with an http URL, will grab from JavaCV IPCameraFrameGrabber
    // if called with "stream", will grab from directory containing a series of images, e.g. saved images
    // otherwise assumes String param is fully qualified path to a single image to load
    public void process(String source) {

    	if (source.equalsIgnoreCase("stream")) {
    		
    		// image 300 shows position at end of auton
    		// image 530 shows successful shot after teleop alignment after auton
    		// images 1088 - 1264 show successful teleop lineup and shot (1262, 1263 show ball movement)
    		// images 1555 - 1761 show lineup that's slightly off on the left
    		process(new savedImageStreamer("d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-5.19-44-24"));
    		
    		// no shot, just turning in auton
    		//process(new savedImageStreamer("d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-5.17-44-47"));
    		
    		// practice field
    		// images 1 - 50 show outerworks shot (I think)
    		// image 499 is end of 2nd outerworks shot, after moving back a little, try looking around 480 and forward for start    		
    		//process(new savedImageStreamer("d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-6.8-23-7"));
    		
    		// practice field
    		// no shots;   looks like closer in than outerworks though...maybe images saved after successful shots taken?
    		//process(new savedImageStreamer("d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-6.8-43-55"));
    		// stops before any shots taken; shows auton drive and turn but no shot
    		//process(new savedImageStreamer("d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-6.15-26-8"));
    	}
    	
    	if (source.startsWith("http:")) {
    		savedFrameCount = 0;
    		setupButtons();
    		while (true) {
    			Mat orig = loadFromJavaCV(source);
    			if (orig == null) {
    				System.out.println("null frame from JavaCV: " + source);
    				try { Thread.sleep(500); } catch (Exception e) {}
    				continue;
    			}
    		}
    	}
    	
    	if (source.startsWith("http:")) {
    		System.out.println("somehow we exited the JavaCV loop for " + source);
    		System.out.println("should not have gotten here...");
    		return;
    	}
    	
    	Mat orig = loadFromFile(source);
    	if (orig == null) {
    		System.out.println("couldn't load " + source);
    		return;
    	}
		Mat b = binarizeSubt(orig);
//		binIcon.setImage(matToBuf(b));
		findTargets(b);
		imgIcon.setImage(drawGreenZone(drawTargets(imgBuf)));
		imgFrame.repaint();
//		binFrame.repaint();
    }
    
    public static void main(String[] args) {
		PracticeBotMain theApp = new PracticeBotMain();
		
		//theApp.process("d:/FRC-2016/ControlsDesign/visionTargeting/RealFullField/7.jpg"); //7.jpg
//		String fileName = "d:/FRC-2016/ControlsDesign/visionTargeting/savedImages-2016-1-29.20-31-57/Camera.2098.jpg";
//		String fileName = "d:/FRC-2016/ControlsDesign/visionTargeting/savedImages-2016-1-29.20-31-57/Camera.2310.jpg";
//		String fileName = "d:/FRC-2016/ControlsDesign/visionTargeting/Camera.634.jpg";
		String fileName = "d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-5.19-44-24/Camera.1263.jpg"; //529
//		String fileName = "d:/FRC-2016/ControlsDesign/visionTargeting/HHImages/savedImages-2016-2-6.8-23-7/Camera.495.jpg"; 
		String webCam = "http://10.26.7.12/mjpg/video.mjpg";
//		theApp.process("stream");
		theApp.process(webCam);
    }

	@Override
	public void mouseClicked(MouseEvent e) {
		pausePlayback = !pausePlayback;
		
	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}


}
