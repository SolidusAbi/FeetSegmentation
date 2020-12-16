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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//VTK Includes
#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>
#include <vtkImageMask.h>
#include <vtkImageResize.h>

#include <vtkPNGWriter.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

//pcl::PointCloud<pcl::PointXYZ> vtkImageToPointCloud(vtkImageData *depthImg)
//{
//  uint16_t *data = reinterpret_cast<uint16_t *>(depthImg->GetScalarPointer());
//  int *dimensions = depthImg->GetDimensions();

//  pcl::PointCloud<pcl::PointXYZ> pointCloud;

//  for (int x = 0; x < dimensions[0]; ++x)
//  {
//    for (int y=0; y < dimensions[1]; ++y)
//    {
//      uint16_t depthPixel = data[ y * dimensions[0] + x];
//      pointCloud.push_back(pcl::PointXYZ(x, y, depthPixel));
//    }
//  }

//  //tmp
//  pcl::io::savePCDFile("test.pcd", pointCloud);
//  return pointCloud;
//}

void applyMask(vtkImageData* depthImg, vtkImageData *mask)
{
  vtkSmartPointer<vtkImageMask> maskFilter =
      vtkSmartPointer<vtkImageMask>::New();

  maskFilter->SetInput1Data(depthImg);
  maskFilter->SetInput2Data(mask);
  maskFilter->Update();
  depthImg->DeepCopy(maskFilter->GetOutput());
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

    vtkSmartPointer<vtkPNGReader> reader =
        vtkSmartPointer<vtkPNGReader>::New();

    const char* filename = "../../../DataTest/Images/ADM001/ADM001_Depth_T0.png";
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
    const char* mask_filename = "../../../TernausNet/result/ADM001/ADM001_RGB_T0.png";
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

    // Apply mask
    applyMask(depthImg, mask);

    // Save image (Testing)
    vtkSmartPointer<vtkPNGWriter> writer =
        vtkSmartPointer<vtkPNGWriter>::New();

    writer->SetFileName("test.png");
    writer->SetInputData(depthImg);
    writer->Write();

    // Generate PointCloud
    PointCloud::Ptr cloud(new PointCloud());

    uint16_t *depthData = reinterpret_cast<uint16_t *>(depthImg->GetScalarPointer());
    size_t currentRow = 0;

    cloud->width = dimensions[0];
    cloud->height = dimensions[1];
    cloud->resize(cloud->width * cloud->height);

    // Lambda function (generatePoint)
    std::function<Point(const size_t &wIdx)> generatePoint =
          [ &depthData, &dimensions, &currentRow ](const size_t &wIdx)
    {
      return Point(wIdx, currentRow, depthData[currentRow * dimensions[0] + wIdx]);
    };

    std::vector<int> pixelsIdx_w(dimensions[0]);
    std::iota(pixelsIdx_w.begin(), pixelsIdx_w.end(), 0);

    for (; currentRow < static_cast<size_t>(dimensions[1]); ++currentRow)
    {
      QFuture<Point> mapper = QtConcurrent::mapped(pixelsIdx_w.begin(), pixelsIdx_w.end(), generatePoint);
      QVector<Point> results = mapper.results().toVector();

      Point *pcData = &(cloud->points.data()[currentRow*dimensions[0]]);
      std::copy(results.begin(), results.end(), pcData);
    }

    // Pass filter (removing 0 values)
    pcl::io::savePCDFile("depth_image.pcd", *cloud);

    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.0);
    pass.setNegative(true);
    pass.filter(*cloud);

    pcl::io::savePCDFile("pass_filtered.pcd", *cloud);

    // Statistical Filter
    std::vector<int> inlierIndices;

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(.001);
    sor.filter(inlierIndices);

    pcl::IndicesPtr indicesPtr(new std::vector<int>(inlierIndices));

    // Create the filtering object
    pcl::ExtractIndices<Point> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (indicesPtr);
    extract.setNegative (false);
    extract.filter (*cloud);

    pcl::io::savePCDFile("statistical_filtered.pcd", *cloud);

    // Plane Segmentation, RANSAC Optimizer
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<Point> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (50);
    seg.setMaxIterations (5000);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    pcl::IndicesPtr indicesPtr2(new std::vector<int>(inliers->indices));

    extract.setInputCloud (cloud);
    extract.setIndices (indicesPtr2);
    extract.setNegative (false);
    extract.filter (*cloud);

    pcl::io::savePCDFile("plane_segmented.pcd", *cloud);

    return a.exec();
}
