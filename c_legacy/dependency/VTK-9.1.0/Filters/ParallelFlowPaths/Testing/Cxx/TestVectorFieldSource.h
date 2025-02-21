#include "vtkImageAlgorithm.h"
#include <vtkInformationVector.h>

class TestVectorFieldSource : public vtkImageAlgorithm
{
public:
  static TestVectorFieldSource* New();
  vtkTypeMacro(TestVectorFieldSource, vtkImageAlgorithm);
  void SetBoundingBox(double x0, double x1, double y0, double y1, double z0, double z1);
  void SetExtent(int xMin, int xMax, int yMin, int yMax, int zMin, int zMax);

protected:
  TestVectorFieldSource();
  ~TestVectorFieldSource();
  int RequestInformation(vtkInformation* request, vtkInformationVector** inputInfoVectors,
    vtkInformationVector* outputInfoVector) override;
  void GetSpacing(double dx[3]);
  void GetSize(double dx[3]);
  void ExecuteDataWithInformation(vtkDataObject* outData, vtkInformation* outInfo) override;

private:
  int Extent[6];
  double BoundingBox[6];
  int Spacing;
};
