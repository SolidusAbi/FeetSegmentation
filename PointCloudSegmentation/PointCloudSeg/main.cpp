//Qt Includes
#include <QCoreApplication>
#include <QtConcurrent>
#include <QFuture>
#include <QRegExp>
#include <QDir>

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


void cloud2vtkImage(PointCloud::Ptr pointCloud, vtkImageData *img)
{
  vtkImageData * depthImg = vtkImageData::New();
//  depthImg->SetDimensions(pointCloud->width, pointCloud->height, 1);
  depthImg->SetDimensions(img->GetDimensions());
  depthImg->SetSpacing(1.0, 1.0, 1.0);
  depthImg->SetOrigin(.0, .0, .0);
  depthImg->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

  size_t nPixels = depthImg->GetDimensions()[0]*depthImg->GetDimensions()[1];
  size_t stride = depthImg->GetDimensions()[0];

  uint16_t data[nPixels];
  std::memset(data, 0, sizeof(uint16_t) * nPixels);

//  pcl::transformPointCloud(*cloud, *cloud, img2pc.inverse());

  for(PointCloud::iterator it = pointCloud->begin(); it!= pointCloud->end(); ++it)
  {
    Point point = *it;
    int x = point._PointXYZ::x;
    int y = point._PointXYZ::y;
    data[y * stride + x] = point._PointXYZ::z;
  }

//  pcl::transformPointCloud(*cloud, *cloud, img2pc);

  std::memcpy(depthImg->GetScalarPointer(), data, sizeof(uint16_t) * nPixels);

  img->DeepCopy(depthImg);
  depthImg->Delete();
}

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

    QDir images_dir("../../../DataTest/Images/");
    QDir mask_dir("../../../TernausNet/result");
    QStringList patients = images_dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);

    QDir::current().mkdir("result/");
    QDir output_dir = QDir::current().filePath("result");

    QRegExp re;
    re.setPattern("[^\\s]+(Depth_T[0-9]+)(\\.(png))$");

    for (QString patient : patients)
    {
      QDir current_path = images_dir.filePath(patient);
      QDir current_mask_path = mask_dir.filePath(patient);

      if (!current_mask_path.exists())
        continue;

      // Crear directorio donde almacenar y preparar el QDir
      output_dir.mkdir(patient);
      QDir patient_result_dir = output_dir.filePath(patient);

      QStringList filenames = current_path.entryList();
      for (QString filename : filenames.filter(re))
      {
        QString mask_filename = filename;
        mask_filename.replace("Depth", "RGB");
        if (!current_mask_path.entryList().contains(mask_filename)) //Si no lo contiene, pasamos al siguiente!
          continue;

        // Load depth Image
        vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
        reader->SetFileName(current_path.filePath(filename).toStdString().c_str());
        reader->Update();

        vtkSmartPointer<vtkImageData> depthImg;
        depthImg = reader->GetOutput();

        // Resize image
        vtkSmartPointer<vtkImageResize> vtkResize = vtkSmartPointer<vtkImageResize>::New();

        vtkResize->SetInputData(depthImg);
        vtkResize->SetOutputDimensions(512, 512, 1);
        vtkResize->Update();

        depthImg->DeepCopy(vtkResize->GetOutput());

        // Load mask
        vtkSmartPointer<vtkPNGReader> mask_reader =
            vtkSmartPointer<vtkPNGReader>::New();
        mask_reader->SetFileName(current_mask_path.filePath(mask_filename).toStdString().c_str());
        mask_reader->Update();

        vtkSmartPointer<vtkImageData> mask;
        mask = mask_reader->GetOutput();

        // Apply mask
        applyMask(depthImg, mask);

        // Generate PointCloud
        PointCloud::Ptr cloud(new PointCloud());

        int *dimensions = mask ->GetDimensions();
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

        cloud2vtkImage(cloud, depthImg);

        // Save image (Testing)
        vtkSmartPointer<vtkPNGWriter> writer =
            vtkSmartPointer<vtkPNGWriter>::New();

        writer->SetFileName(patient_result_dir.filePath(filename).toStdString().c_str());
        writer->SetInputData(depthImg);
        writer->Write();
      }
    }

    qDebug() << "He terminado!!";

//    vtkSmartPointer<vtkPNGReader> reader =
//        vtkSmartPointer<vtkPNGReader>::New();

//    const char* filename = "../../../DataTest/Images/ADM001/ADM001_Depth_T0.png";
//    reader->SetFileName(filename);
//    reader->Update();

//    vtkSmartPointer<vtkImageData> depthImg;
//    depthImg = reader->GetOutput();

//    // Resize image
//    vtkSmartPointer<vtkImageResize> vtkResize =
//         vtkSmartPointer<vtkImageResize>::New();

//    vtkResize->SetInputData(depthImg);
//    vtkResize->SetOutputDimensions(512, 512, 1);
//    vtkResize->Update();

//    depthImg->DeepCopy(vtkResize->GetOutput());

//    // Load mask
//    const char* mask_filename = "../../../TernausNet/result/ADM001/ADM001_RGB_T0.png";
//    vtkSmartPointer<vtkPNGReader> mask_reader =
//        vtkSmartPointer<vtkPNGReader>::New();
//    mask_reader->SetFileName(mask_filename);
//    mask_reader->Update();

//    vtkSmartPointer<vtkImageData> mask;
//    mask = mask_reader->GetOutput();
//    int *dimensions = mask ->GetDimensions();
//    std::cout << dimensions[0] << std::endl;
//    std::cout << dimensions[1] << std::endl;
//    std::cout << dimensions[2] << std::endl;

//    // Apply mask
//    applyMask(depthImg, mask);

//    // Save image (Testing)
//    vtkSmartPointer<vtkPNGWriter> writer =
//        vtkSmartPointer<vtkPNGWriter>::New();

//    writer->SetFileName("test.png");
//    writer->SetInputData(depthImg);
//    writer->Write();

//    // Generate PointCloud
//    PointCloud::Ptr cloud(new PointCloud());

//    uint16_t *depthData = reinterpret_cast<uint16_t *>(depthImg->GetScalarPointer());
//    size_t currentRow = 0;

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

//    // Pass filter (removing 0 values)
//    pcl::io::savePCDFile("depth_image.pcd", *cloud);

//    pcl::PassThrough<Point> pass;
//    pass.setInputCloud(cloud);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0, 0.0);
//    pass.setNegative(true);
//    pass.filter(*cloud);

//    pcl::io::savePCDFile("pass_filtered.pcd", *cloud);

//    // Statistical Filter
//    std::vector<int> inlierIndices;

//    pcl::StatisticalOutlierRemoval<Point> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK(50);
//    sor.setStddevMulThresh(.001);
//    sor.filter(inlierIndices);

//    pcl::IndicesPtr indicesPtr(new std::vector<int>(inlierIndices));

//    // Create the filtering object
//    pcl::ExtractIndices<Point> extract;
//    // Extract the inliers
//    extract.setInputCloud (cloud);
//    extract.setIndices (indicesPtr);
//    extract.setNegative (false);
//    extract.filter (*cloud);

//    pcl::io::savePCDFile("statistical_filtered.pcd", *cloud);

//    // Plane Segmentation, RANSAC Optimizer
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//    // Create the segmentation object
//    pcl::SACSegmentation<Point> seg;

//    // Optional
//    seg.setOptimizeCoefficients (true);

//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (50);
//    seg.setMaxIterations (5000);

//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);

//    pcl::IndicesPtr indicesPtr2(new std::vector<int>(inliers->indices));

//    extract.setInputCloud (cloud);
//    extract.setIndices (indicesPtr2);
//    extract.setNegative (false);
//    extract.filter (*cloud);

//    pcl::io::savePCDFile("plane_segmented.pcd", *cloud);

//    cloud2vtkImage(cloud, depthImg);
//    writer->Write();

    return a.exec();
}
