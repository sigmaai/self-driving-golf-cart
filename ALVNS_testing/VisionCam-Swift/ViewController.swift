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
    @IBOutlet weak var label: UILabel!
    
    private var model = steering_model_v4()
    
    override func viewDidLoad(){
        super.viewDidLoad()
        setupAVCapture()
//        let image = UIImage.init(named: "1475187062173202962.jpg")
//        let cvBuffer = buffer(from: image!)
//        predictSteeringAngle(input: cvBuffer)
    }
    
    func buffer(from image: UIImage) -> CVPixelBuffer? {
        let attrs = [kCVPixelBufferCGImageCompatibilityKey: kCFBooleanTrue, kCVPixelBufferCGBitmapContextCompatibilityKey: kCFBooleanTrue] as CFDictionary
        var pixelBuffer : CVPixelBuffer?
        let status = CVPixelBufferCreate(kCFAllocatorDefault, Int(image.size.width), Int(image.size.height), kCVPixelFormatType_32ARGB, attrs, &pixelBuffer)
        guard (status == kCVReturnSuccess) else {
            return nil
        }
        
        CVPixelBufferLockBaseAddress(pixelBuffer!, CVPixelBufferLockFlags(rawValue: 0))
        let pixelData = CVPixelBufferGetBaseAddress(pixelBuffer!)
        
        let rgbColorSpace = CGColorSpaceCreateDeviceRGB()
        let context = CGContext(data: pixelData, width: Int(image.size.width), height: Int(image.size.height), bitsPerComponent: 8, bytesPerRow: CVPixelBufferGetBytesPerRow(pixelBuffer!), space: rgbColorSpace, bitmapInfo: CGImageAlphaInfo.noneSkipFirst.rawValue)
        
        context?.translateBy(x: 0, y: image.size.height)
        context?.scaleBy(x: 1.0, y: -1.0)
        
        UIGraphicsPushContext(context!)
        image.draw(in: CGRect(x: 0, y: 0, width: image.size.width, height: image.size.height))
        UIGraphicsPopContext()
        CVPixelBufferUnlockBaseAddress(pixelBuffer!, CVPixelBufferLockFlags(rawValue: 0))
        
        return pixelBuffer
    }
    
    override func didReceiveMemoryWarning(){
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func predictSteeringAngle(input: CVPixelBuffer?){

        guard let prediction = try? model.prediction(image: input!) else {
            return
        }
        DispatchQueue.main.async { // perform all UI updates on the main queue
            self.label.text = "\(prediction.angle[0])"
            let tr = CGAffineTransform.identity.rotated(by: -CGFloat.init(prediction.angle[0]))
            self.label.transform = tr
        }
        print(prediction.angle)
    }
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection)
    {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }
        
        let ciimage : CIImage = CIImage(cvPixelBuffer: pixelBuffer)
        let image : UIImage = self.convert(cmage: ciimage)
        let resized = resizeImage(image: image, newWidth: 880)
        //let cropped  = cropToBounds(image: resized!, width: 640, height: 480)
        let cropped = cropImage(imageToCrop: resized!, toRect: CGRect.init(x: (resized?.size.width)! / 4, y: 0.0, width: 640, height: 480))
        let cvBuffer = buffer(from: cropped)
        predictSteeringAngle(input: cvBuffer)
    }
    
    func cropImage(imageToCrop:UIImage, toRect rect:CGRect) -> UIImage{
        
        let imageRef:CGImage = imageToCrop.cgImage!.cropping(to: rect)!
        let cropped:UIImage = UIImage(cgImage:imageRef)
        return cropped
    }
    
    func cropToBounds(image: UIImage, width: Double, height: Double) -> UIImage {
        
        let contextImage: UIImage = UIImage.init(cgImage: image.cgImage!)
        
        let contextSize: CGSize = contextImage.size
        
        var posX: CGFloat = 0.0
        var posY: CGFloat = 0.0
        var cgwidth: CGFloat = CGFloat(width)
        var cgheight: CGFloat = CGFloat(height)
        
        // See what size is longer and create the center off of that
        if contextSize.width > contextSize.height {
            posX = ((contextSize.width - contextSize.height) / 2)
            posY = 0
            cgwidth = contextSize.height
            cgheight = contextSize.height
        } else {
            posX = 0
            posY = ((contextSize.height - contextSize.width) / 2)
            cgwidth = contextSize.width
            cgheight = contextSize.width
        }
        
        let rect: CGRect = CGRect.init(x: posX, y: posY, width: cgwidth, height: cgheight)
        
        // Create bitmap image from context using the rect
        let imageRef: CGImage = (contextImage.cgImage?.cropping(to: rect))!
        // Create a new image based on the imageRef and rotate back to the original orientation
        let image = UIImage.init(cgImage: imageRef, scale: image.scale, orientation: image.imageOrientation)
        return image
    }
    
    func resizeImage(image: UIImage, newWidth: CGFloat) -> UIImage? {
        
        let scale = newWidth / image.size.width
        let newHeight = image.size.height * scale
        UIGraphicsBeginImageContext(CGSize.init(width: newWidth, height: newHeight))
        image.draw(in: CGRect.init(x: 0, y: 0, width: newWidth, height: newHeight))
        let newImage = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        
        return newImage
    }

    
    // Convert CIImage to CGImage
    func convert(cmage:CIImage) -> UIImage{
        
        let context:CIContext = CIContext.init(options: nil)
        let cgImage:CGImage = context.createCGImage(cmage, from: cmage.extent)!
        let image:UIImage = UIImage.init(cgImage: cgImage)
        return image
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
        
        if let connection =  self.previewLayer?.connection  {
            
            let currentDevice: UIDevice = UIDevice.current
            
            let orientation: UIDeviceOrientation = currentDevice.orientation
            
            let previewLayerConnection : AVCaptureConnection = connection
            
            if previewLayerConnection.isVideoOrientationSupported {
                
                switch (orientation) {
                case .portrait: updatePreviewLayer(layer: previewLayerConnection, orientation: .portrait)
                
                    break
                    
                case .landscapeRight: updatePreviewLayer(layer: previewLayerConnection, orientation: .landscapeLeft)
                
                    break
                    
                case .landscapeLeft: updatePreviewLayer(layer: previewLayerConnection, orientation: .landscapeRight)
                
                    break
                    
                case .portraitUpsideDown: updatePreviewLayer(layer: previewLayerConnection, orientation: .portraitUpsideDown)
                
                    break
                    
                default: updatePreviewLayer(layer: previewLayerConnection, orientation: .landscapeRight)
                
                    break
                }
            }
        }
        session.startRunning()
    }
    
    private func updatePreviewLayer(layer: AVCaptureConnection, orientation: AVCaptureVideoOrientation) {
        
        layer.videoOrientation = orientation
        
        previewLayer.frame = self.view.bounds
        
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
    
    func captureOutput(_ captureOutput: AVCaptureOutput, didDrop didDropSampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        // print("frame dropped")
    }
}

