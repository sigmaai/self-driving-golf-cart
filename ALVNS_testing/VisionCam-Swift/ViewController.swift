//
//  ViewController.swift
//  VisionCam-Swift
//
//  Created by Frank Doepke on 5/24/17.
//  Copyright © 2017 Apple. All rights reserved.
//

import UIKit
import AVFoundation
import Vision

class ViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate {
    
    @IBOutlet weak private var previewView: UIView!
    
    private let session = AVCaptureSession()
    
    private var previewLayer:AVCaptureVideoPreviewLayer! = nil
    private let videoDataOutput = AVCaptureVideoDataOutput()
    
    private let videoDataOutputQueue = DispatchQueue(label: "VideoDataOutputQueue", qos: .userInitiated)
    
    private var bufferSize:CGSize = .zero
    private var textLayer:CATextLayer! = nil
    private var rootLayer:CALayer! = nil
    private var detectionOverlay:CALayer! = nil
    private var detectedRectangleLayer:CAShapeLayer! = nil
    
    // Vision parts
    private var requests = [VNRequest]()
    
    func setupVision()
    {
        // Setup Vision parts
        // Create the model for the CoreML based request
        guard let steeringModel = try? VNCoreMLModel(for: steering_v1().model)
            else { fatalError("can't load Vision ML model")}
        let steeringRequest = VNCoreMLRequest(model: steeringModel, completionHandler: handleSteeringOutput)
        steeringRequest.imageCropAndScaleOption = VNImageCropAndScaleOption.centerCrop
        
        let rectangleDetectionRequest = VNDetectRectanglesRequest(completionHandler:self.handleRectangles)
        rectangleDetectionRequest.minimumSize = 0.1
        rectangleDetectionRequest.maximumObservations = 20
        
        self.requests = [rectangleDetectionRequest, steeringRequest]
    }
    
    func handleSteeringOutput(request: VNRequest, error: Error?) {
        guard let observations = request.results
            else { print("no results: \(error!)"); return }
        
        let classifications = observations[0...4] // use up to top 4 results
            .flatMap({ $0 as? VNClassificationObservation }) // ignore unexpected classes
            .filter({ $0.confidence > 0.3 }) // skip low-confidence classifications
            .map(self.textForClassification) // extract displayable text
        
        DispatchQueue.main.async { // perform all UI updates on the main queue
            self.textLayer.string = "Classification: \n" + classifications.joined(separator: "\n")
        }
    }
    func handleRectangles(request: VNRequest, error: Error?) {
        DispatchQueue.main.async {
            // perform all the UI updates on the main queue
            self.drawVisionRequestResults(request.results as! [VNObservation])
        }
    }
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection)
    {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }
        var requestOptions:[VNImageOption : Any] = [:]
        
        if let cameraIntrinsicData = CMGetAttachment(sampleBuffer, kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, nil) {
            requestOptions = [.cameraIntrinsics:cameraIntrinsicData]
        }
        
        let exifOrientation = self.exifOrientationFromDeviceOrientation()
        
        let imageRequestHandler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: CGImagePropertyOrientation(rawValue: UInt32(exifOrientation))!, options: requestOptions)
        do {
            try imageRequestHandler.perform(self.requests)
            
        } catch {
            print(error)
        }
    }
    
    func drawVisionRequestResults(_ results:[VNObservation])
    {
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey:kCATransactionDisableActions)
        
        detectedRectangleLayer.isHidden = true // hide by default - it will be 'unhidden' while iterating through the observations
        
        let boxOutline = CGMutablePath()
        
        for observation in results
        {
            if observation is VNRectangleObservation
            {
                let rectangleObservation = observation as! VNRectangleObservation
                var topLeft = rectangleObservation.topLeft
                var topRight = rectangleObservation.topRight
                var bottomLeft = rectangleObservation.bottomLeft
                var bottomRight = rectangleObservation.bottomRight
                
                // scale to previewLayer
                topLeft.x *= bufferSize.width
                topLeft.y *= bufferSize.height
                topRight.x *= bufferSize.width
                topRight.y *= bufferSize.height
                bottomLeft.x *= bufferSize.width
                bottomLeft.y *= bufferSize.height
                bottomRight.x *= bufferSize.width
                bottomRight.y *= bufferSize.height
                
                boxOutline.move(to: topLeft)
                boxOutline.addLine(to: topRight)
                boxOutline.addLine(to: bottomRight)
                boxOutline.addLine(to: bottomLeft)
                boxOutline.addLine(to: topLeft)
            }
        }
        detectedRectangleLayer.path = boxOutline
        detectedRectangleLayer.isHidden = false;
        self.updateLayerGeometry()
        CATransaction.commit()
    }
    
    func textForClassification(_ classification: VNClassificationObservation) -> String {
        // strip off prefix from classification identifier as it is just a non-human-readable identifier
        let index = classification.identifier.index(classification.identifier.startIndex, offsetBy: 10)
        let readableIdentifier = classification.identifier.substring(from: index)
        return "\(readableIdentifier) \n     Confidence: \(String(format: "%.2f", classification.confidence)) \n"
    }
    
    override func viewDidLoad(){
        super.viewDidLoad()
        setupAVCapture()
    }
    
    override func didReceiveMemoryWarning(){
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func setupAVCapture(){
        
        var deviceInput: AVCaptureDeviceInput!
        
        // Select a video device, make an input
        let videoDevice = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera], mediaType: .video, position: .back).devices.first
        do {
            deviceInput = try AVCaptureDeviceInput(device: videoDevice!)
        } catch {
            print("Could not create video device input: \(error)")
            return
        }
        
        session.beginConfiguration()
        session.sessionPreset = .high
        
        // Add a video input
        guard session.canAddInput(deviceInput) else {
            print("Could not add video device input to the session")
            session.commitConfiguration()
            return
        }
        guard session.canAddInput(deviceInput) else {
            print("Could not add video device input to the session")
            session.commitConfiguration()
            return
        }
        session.addInput(deviceInput)
        
        if session.canAddOutput(videoDataOutput) {
            session.addOutput(videoDataOutput)
            // Add a video data output
            videoDataOutput.alwaysDiscardsLateVideoFrames = true
            videoDataOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)]
            videoDataOutput.setSampleBufferDelegate(self, queue: videoDataOutputQueue)
        } else {
            print("Could not add video data output to the session")
            session.commitConfiguration()
            return
        }
        let captureConnection = videoDataOutput.connection(with: .video)
        captureConnection?.isEnabled = true    //always process the frames
        // check for Camera intrinsics
        captureConnection?.isCameraIntrinsicMatrixDeliveryEnabled = true // this is not supported on all devices. Use cameraIntrinsicMatrixDeliverySupported to check for availability
        
        
        do {
            try  videoDevice!.lockForConfiguration()
            let format = videoDevice?.activeFormat
            let fdesc = format!.formatDescription
            let dims = CMVideoFormatDescriptionGetDimensions(fdesc)
            bufferSize.width = CGFloat(dims.width)
            bufferSize.height = CGFloat(dims.height)
            videoDevice!.unlockForConfiguration()
            
        } catch {
            print(error)
        }
        
        session.commitConfiguration()
        
        previewLayer = AVCaptureVideoPreviewLayer(session: session)
        previewLayer.name = "CameraPreview"
        previewLayer.backgroundColor = UIColor.black.cgColor
        previewLayer.videoGravity = AVLayerVideoGravity.resizeAspectFill
        rootLayer = previewView.layer
        rootLayer.masksToBounds = true
        previewLayer.frame = rootLayer.bounds
        rootLayer.addSublayer(previewLayer)
        
        self.setupLayers()
        self.updateLayerGeometry()
        
        self.setupVision()
        session.startRunning()
    }
    
    // clean up capture setup
    func teardownAVCapture()
    {
        previewLayer.removeFromSuperlayer();
        previewLayer = nil
    }
    
    func setupLayers()
    {
        detectionOverlay = CALayer() // container layer that has all the renderings of the observations
        detectionOverlay.name = "DetectionOverlay"
        detectionOverlay.masksToBounds = true
        detectionOverlay.anchorPoint = CGPoint(x:0.5, y:0.5)
        detectionOverlay.bounds = CGRect(x: 0.0, y: 0.0, width: bufferSize.width, height:bufferSize.height) // we make the layer for the overlay drawing the same size as the buffers going into VN to make drawing the results easier in pixel space
        detectionOverlay.position = CGPoint(x:rootLayer.bounds.midX, y:rootLayer.bounds.midY)
        
        detectedRectangleLayer = CAShapeLayer()
        detectedRectangleLayer.name = "RectangleOutlineLayer"
        detectedRectangleLayer.bounds = CGRect(x: 0.0, y: 0.0, width: bufferSize.width, height:bufferSize.height)
        detectedRectangleLayer.anchorPoint = CGPoint(x:0.5, y:0.5)
        detectedRectangleLayer.position = CGPoint(x:detectionOverlay.bounds.midX, y:detectionOverlay.bounds.midY)
        let fillColor = CGColor.init(colorSpace: CGColorSpaceCreateDeviceRGB(), components: [1.0, 1.0, 0.0, 0.3])
        detectedRectangleLayer.fillColor = fillColor
        detectionOverlay.addSublayer(detectedRectangleLayer)
        rootLayer.addSublayer(detectionOverlay)
        
        let textLayerRect = rootLayer.bounds.insetBy(dx: 20.0, dy: 20.0)
        textLayer = CATextLayer();
        textLayer.name = "TextOverlay"
        textLayer.frame = textLayerRect
        textLayer.shadowOpacity = 0.7
        textLayer.contentsGravity = kCAGravityTopLeft
        textLayer.fontSize = 24.0
        textLayer.contentsScale = 2.0 // retina rendering
        rootLayer.addSublayer(textLayer)
    }
    
    func updateLayerGeometry()
    {
        let bounds = rootLayer.bounds
        var x:CGFloat, y:CGFloat
        
        x = bounds.size.width / bufferSize.width
        y = bounds.size.height / bufferSize.height
        
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey:kCATransactionDisableActions)
        
        // rotate the layer into screen orientation and scale and mirror
        detectionOverlay.setAffineTransform(CGAffineTransform(scaleX: x, y:-y))
        
        detectionOverlay.position = CGPoint (x: bounds.midX, y: bounds.midY)
        
        CATransaction.commit()
        
    }
    
    func exifOrientationFromDeviceOrientation() -> Int32
    {
        
        let curDeviceOrientation = UIDevice.current.orientation
        let exifOrientation:Int32
        
        /* kCGImagePropertyOrientation values
         The intended display orientation of the image. If present, this key is a CFNumber value with the same value as defined
         by the TIFF and EXIF specifications -- see enumeration of integer constants.
         The value specified where the origin (0,0) of the image is located. If not present, a value of 1 is assumed.
         
         used when calling featuresInImage: options: The value for this key is an integer NSNumber from 1..8 as found in kCGImagePropertyOrientation.
         If present, the detection will be done based on that orientation but the coordinates in the returned features will still be based on those of the image. */
        /*
         enum {
         EXIF_0ROW_TOP_0COL_LEFT            = 1, //   1  =  0th row is at the top, and 0th column is on the left (THE DEFAULT).
         EXIF_0ROW_TOP_0COL_RIGHT            = 2, //   2  =  0th row is at the top, and 0th column is on the right.
         EXIF_0ROW_BOTTOM_0COL_RIGHT      = 3, //   3  =  0th row is at the bottom, and 0th column is on the right.
         EXIF_0ROW_BOTTOM_0COL_LEFT       = 4, //   4  =  0th row is at the bottom, and 0th column is on the left.
         EXIF_0ROW_LEFT_0COL_TOP          = 5, //   5  =  0th row is on the left, and 0th column is the top.
         EXIF_0ROW_RIGHT_0COL_TOP         = 6, //   6  =  0th row is on the right, and 0th column is the top.
         EXIF_0ROW_RIGHT_0COL_BOTTOM      = 7, //   7  =  0th row is on the right, and 0th column is the bottom.
         EXIF_0ROW_LEFT_0COL_BOTTOM       = 8  //   8  =  0th row is on the left, and 0th column is the bottom.
         };
         */
        
        switch curDeviceOrientation {
        case UIDeviceOrientation.portraitUpsideDown:  // Device oriented vertically, home button on the top
            exifOrientation = 8
        case UIDeviceOrientation.landscapeLeft:       // Device oriented horizontally, home button on the right
            exifOrientation = 1
        case UIDeviceOrientation.landscapeRight:      // Device oriented horizontally, home button on the left
            exifOrientation = 3
        case UIDeviceOrientation.portrait:            // Device oriented vertically, home button on the bottom
            exifOrientation = 6
        default:
            exifOrientation = 6                       // assume portrait when held flat
        }
        return exifOrientation
    }
    
    
    func captureOutput(_ captureOutput: AVCaptureOutput, didDrop didDropSampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        
        // print("frame dropped")
    }
}

