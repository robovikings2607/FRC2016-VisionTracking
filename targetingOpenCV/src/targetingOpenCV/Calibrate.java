package targetingOpenCV;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Calibrate {
/*
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
*/
	private List<Mat> imagePoints, objectPoints;
	private JFrame imgFrame; 
	private ImageIcon imgIcon;
	private Mat intrinsic, distCoeffs;
	private Size imageSize;
	
	public Mat getCameraMatrix() {
		return intrinsic;
	}
	
	public Mat getDistortMatrix() {
		return distCoeffs;
	}
	
	public Calibrate() {
		imagePoints = new ArrayList<>();
		objectPoints = new ArrayList<>();
		intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		distCoeffs = new Mat();
		imgFrame = new JFrame("Chessboard");
		imgFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		imgIcon = new ImageIcon();
		imgFrame.getContentPane().add(new JLabel(imgIcon));
		imgFrame.pack();
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

	private void processSnapshot(String imageFileName, int horizCorners, int vertCorners) {
		
		Mat origImg = Imgcodecs.imread(imageFileName);
		if (origImg == null) {
			System.out.println("ABORTING: Failed to open snapshot " + imageFileName);
			System.exit(0);
		}
		
		Mat greyImg = new Mat();
		Imgproc.cvtColor(origImg, greyImg, Imgproc.COLOR_BGR2GRAY);
		
		MatOfPoint2f imageCorners = new MatOfPoint2f();
		MatOfPoint3f obj = new MatOfPoint3f();
		for (int i = 0; i < horizCorners * vertCorners; i++) {
			obj.push_back(new MatOfPoint3f(new Point3(i / horizCorners, i % vertCorners, 0.0f)));
		}
		Size boardSize = new Size(horizCorners, vertCorners);
		boolean allFound = 
				Calib3d.findChessboardCorners(greyImg, boardSize, imageCorners, 
						Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE);
	  	Calib3d.drawChessboardCorners(origImg, boardSize, imageCorners, allFound);
		imgIcon.setImage(matToBuf(origImg));
		imgFrame.setSize(origImg.width(), origImg.height());
		imgFrame.setVisible(true);
		imgFrame.repaint();
		if (allFound) {
			TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
			Imgproc.cornerSubPix(greyImg, imageCorners, new Size(11, 11), new Size(-1, -1), term);
			imagePoints.add(imageCorners);
			objectPoints.add(obj);
			imageSize = greyImg.size();
		} else {
			System.out.println("ABORTING:  Failed to find " + horizCorners + "x" + vertCorners + " in snapshot: " + imageFileName);
			System.exit(0);
		}
		
	}
	
	private void calibrateCamera() {
		List<Mat> rvecs = new ArrayList<>();
		List<Mat> tvecs = new ArrayList<>();
		intrinsic.put(0, 0, 1);
		intrinsic.put(1, 1, 1);
		// calibrate!
		Calib3d.calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic, distCoeffs, rvecs, tvecs);		
	}
	
	private void showUndistorted(String fileName) {
		Mat origImg = Imgcodecs.imread(fileName);
		Mat undistorted = new Mat();
		Imgproc.undistort(origImg, undistorted, intrinsic, distCoeffs);
		imgIcon.setImage(matToBuf(undistorted));
		imgFrame.repaint();
		Imgcodecs.imwrite("undistored.jpg", undistorted);
	}
	
	public void performCalibration() {
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image01.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image02.jpg", 6, 9);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image03.jpg", 6, 9);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image04.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image05.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image06.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image07.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image08.jpg", 6, 9);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image09.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image10.jpg", 6, 9);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image11.jpg", 9, 6);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image12.jpg", 6, 9);
		processSnapshot("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image02.jpg", 6, 9);
		calibrateCamera();
//		showUndistorted("d:/FRC-2016/ControlsDesign/M1013-calibFiles/image04.jpg");		
	}
	
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Calibrate calibApp = new Calibrate();
		calibApp.performCalibration();
		
		
	}

	/*
	  	Calib3d.drawChessboardCorners(origImg, boardSize, imageCorners, allFound);
		imgIcon.setImage(matToBuf(origImg));
		imgFrame.setSize(origImg.width(), origImg.height());
		imgFrame.setVisible(true);
		imgFrame.repaint();

	 */
}


