Images images;
RollingAverage speed;
RollingAverage deviation;
moduleSettings mset;

SystemPtr FlirSystem;
CameraList camera_list;
CameraPtr camera_pointer;

void newPattern(){
    images.pattern_image = images.current_image.clone();
    cv::imwrite("Pattern_new.Bmp", images.pattern_image);
}

int get_new_image (CameraPtr pCam){
    try
    {
        ImagePtr convertedImage = pCam->GetNextImage(120);
        if (convertedImage->IsIncomplete())
        {
            //std::cout << "Image incomplete..." << std::endl;
            return 1;
        }
        else{
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize  = convertedImage->GetWidth();
            unsigned int colsize  = convertedImage->GetHeight();

            cv::Mat sample( colsize+YPadding, 
                            rowsize+XPadding, 
                            CV_8UC3, 
                            convertedImage->GetData(), 
                            convertedImage->GetStride());

            images.current_image = sample.clone();
            images.c_stamp = std::chrono::system_clock::now();
        }
        convertedImage->Release();

    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

int startCamera(moduleSettings mset){
    images.pattern_image = cv::imread("Pattern_new.Bmp");

    FlirSystem = System::GetInstance();
    camera_list = FlirSystem->GetCameras();
    const unsigned int numCameras = camera_list.GetSize();

    if (numCameras == 0)
    {
        std::cout << "no cameras" << std::endl;
        return -1;
    }

    camera_pointer = nullptr;
    int result = 0;
    camera_pointer = camera_list.GetBySerial(mset.serial_number);

    if (camera_pointer == nullptr){
        std::cout << "camera connection failure" << std::endl;
        return -1;
    }

    camera_pointer->Init();
    INodeMap& nodeMapTLDevice = camera_pointer->GetTLDeviceNodeMap();
    INodeMap& nodeMap = camera_pointer->GetNodeMap();

    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    ptrAcquisitionMode->SetIntValue(ptrAcquisitionModeContinuous->GetValue());
    camera_pointer->BeginAcquisition();
    
    camera_pointer->AcquisitionFrameRate.SetValue(32); //34.995
    
    std::cout << "acquiring" << std::endl;

    get_new_image(camera_pointer);
    get_new_image(camera_pointer);

    return 0;
}
void stopCamera(){
    camera_pointer->EndAcquisition();
    camera_pointer->DeInit();
    camera_pointer = nullptr;
    camera_list.Clear();
    FlirSystem->ReleaseInstance();
    
    std::cout << "Camera shutdown: Success" << std::endl;
}