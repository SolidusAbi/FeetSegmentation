//Qt Includes
#include <QCoreApplication>
#include <QFuture>
#include <QtConcurrent>

//PCL Includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

//VTK Includes
#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>
#include <vtkImageMask.h>
#include <vtkImageResize.h>

#include <vtkPNGWriter.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

void applyMask(vtkImageData* depthImg, vtkImageData *mask)
{
  vtkSmartPointer<vtkImageMask> maskFilter =
      vtkSmartPointer<vtkImageMask>::New();

  maskFilter->SetInput1Data(depthImg);
  maskFilter->SetInput2Data(mask);
  maskFilter->Update();
  depthImg->DeepCopy(maskFilter->GetOutput());


//  vtkImageToPointCloud(depthImg);
//  pcl::transformPointCloud(*cloud, *cloud, img2pc.inverse());

//  // It is necessary to remove the points with Z value to 0
//  pcl::PassThrough<Point> pass;
//  pass.setInputCloud(cloud);
//  pass.setFilterFieldName("z");
//  pass.setFilterLimits(0.0, 0.0);
//  pass.setNegative(true);
//  pass.filter(*cloud);

//  pcl::transformPointCloud(*cloud, *cloud, img2pc);

//  // Tmp
//  pcl::io::savePCDFile("maskedPCD.pcd", *cloud);
}

vtkImageData * loadMask(const char* filename)
{
  vtkSmartPointer<vtkPNGReader> mask_reader =
      vtkSmartPointer<vtkPNGReader>::New();
  mask_reader->SetFileName(filename);
  mask_reader->Update();

  vtkSmartPointer<vtkImageData> mask;
  mask = mask_reader->GetOutput();

  return mask;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    PointCloud::Ptr cloud(new PointCloud());

    vtkSmartPointer<vtkPNGReader> reader =
        vtkSmartPointer<vtkPNGReader>::New();

    const char* filename = "/home/abian/Workspace/Thesis/Segmentation/DataTest/Images/ADM001/ADM001_Depth_T0.png";
    reader->SetFileName(filename);
    reader->Update();

    vtkSmartPointer<vtkImageData> depthImg;
    depthImg = reader->GetOutput();

    // Resize image
    vtkSmartPointer<vtkImageResize> vtkResize =
         vtkSmartPointer<vtkImageResize>::New();

    vtkResize->SetInputData(depthImg);
    vtkResize->SetOutputDimensions(512, 512, 1);
    vtkResize->Update();

    depthImg->DeepCopy(vtkResize->GetOutput());

    // Load mask
    const char* mask_filename = "/home/abian/Workspace/Thesis/Segmentation/TernausNet/result/ADM001/ADM001_RGB_T0.png";
    vtkSmartPointer<vtkPNGReader> mask_reader =
        vtkSmartPointer<vtkPNGReader>::New();
    mask_reader->SetFileName(mask_filename);
    mask_reader->Update();

    vtkSmartPointer<vtkImageData> mask;
    mask = mask_reader->GetOutput();
    int *dimensions = mask ->GetDimensions();
    std::cout << dimensions[0] << std::endl;
    std::cout << dimensions[1] << std::endl;
    std::cout << dimensions[2] << std::endl;

//    for(int x = 0; x < dimensions[0]; x++)
//    {
//      for(int y = 0; y < dimensions[1]; y++)
//      {
//        uint8_t* pixel = reinterpret_cast<uint8_t*>(mask->GetScalarPointer(x,y,0));
//        if (*pixel > 0)
//          std::cout << *pixel << std::endl;
//      }
//    }

    // Apply mask
    applyMask(depthImg, mask);

    // Save image (Testing)
    vtkSmartPointer<vtkPNGWriter> writer =
        vtkSmartPointer<vtkPNGWriter>::New();

    writer->SetFileName("test.png");
    writer->SetInputData(depthImg);
    writer->Write();



//    int *dimensions = img->GetDimensions();
//    uint16_t *depthData = reinterpret_cast<uint16_t *>(img->GetScalarPointer());
//    size_t currentRow = 0;

//    std::cout << dimensions[0] << std::endl;
//    std::cout << dimensions[1] << std::endl;
//    cloud->width = dimensions[0];
//    cloud->height = dimensions[1];
//    cloud->resize(cloud->width * cloud->height);


//    // Lambda function (generatePoint)
//    std::function<Point(const size_t &wIdx)> generatePoint =
//          [ &depthData, &dimensions, &currentRow ](const size_t &wIdx)
//    {
//      return Point(wIdx, currentRow, depthData[currentRow * dimensions[0] + wIdx]);
//    };

//    std::vector<int> pixelsIdx_w(dimensions[0]);
//    std::iota(pixelsIdx_w.begin(), pixelsIdx_w.end(), 0);

//    for (; currentRow < static_cast<size_t>(dimensions[1]); ++currentRow)
//    {
//      QFuture<Point> mapper = QtConcurrent::mapped(pixelsIdx_w.begin(), pixelsIdx_w.end(), generatePoint);
//      QVector<Point> results = mapper.results().toVector();

//      Point *pcData = &(cloud->points.data()[currentRow*dimensions[0]]);
//      std::copy(results.begin(), results.end(), pcData);
//    }

//    pcl::io::savePCDFile("depth_image.pcd", *cloud);

//    pcl::PassThrough<Point> pass;
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0, 0.0);
//    pass.setNegative(true);
//    pass.filter(*cloud);

//    pcl::io::savePCDFile("pass_filtered.pcd", *cloud);

    return a.exec();
}
