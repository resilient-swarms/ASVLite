/*=========================================================================

  Program:   Visualization Toolkit
  Module:    ObjectFactory.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkDebugLeaks.h"
#include "vtkObjectFactory.h"
#include "vtkObjectFactoryCollection.h"
#include "vtkOutputWindow.h"
#include "vtkOverrideInformation.h"
#include "vtkOverrideInformationCollection.h"
#include "vtkPoints.h"
#include "vtkVersion.h"

static int failed = 0;

class vtkTestPoints : public vtkPoints
{
public:
  // Methods from vtkObject
  ~vtkTestPoints() override = default;

  vtkTypeMacro(vtkTestPoints, vtkPoints);
  static vtkTestPoints* New() { VTK_STANDARD_NEW_BODY(vtkTestPoints); }
  vtkTestPoints() = default;

private:
  vtkTestPoints(const vtkTestPoints&) = delete;
  vtkTestPoints& operator=(const vtkTestPoints&) = delete;
};

class vtkTestPoints2 : public vtkPoints
{
public:
  ~vtkTestPoints2() override = default;

  // Methods from vtkObject
  vtkTypeMacro(vtkTestPoints2, vtkPoints);
  static vtkTestPoints2* New() { VTK_STANDARD_NEW_BODY(vtkTestPoints2); }
  vtkTestPoints2() = default;

private:
  vtkTestPoints2(const vtkTestPoints2&) = delete;
  vtkTestPoints2& operator=(const vtkTestPoints2&) = delete;
};

VTK_CREATE_CREATE_FUNCTION(vtkTestPoints);
VTK_CREATE_CREATE_FUNCTION(vtkTestPoints2);

class VTK_EXPORT TestFactory : public vtkObjectFactory
{
public:
  TestFactory();
  static TestFactory* New()
  {
    TestFactory* f = new TestFactory;
    f->InitializeObjectBase();
    return f;
  }
  const char* GetVTKSourceVersion() override { return VTK_SOURCE_VERSION; }
  const char* GetDescription() override { return "A fine Test Factory"; }

protected:
  TestFactory(const TestFactory&) = delete;
  TestFactory& operator=(const TestFactory&) = delete;
};

TestFactory::TestFactory()
{
  this->RegisterOverride("vtkPoints", "vtkTestPoints", "test vertex factory override", 1,
    vtkObjectFactoryCreatevtkTestPoints);
  this->RegisterOverride("vtkPoints", "vtkTestPoints2", "test vertex factory override 2", 0,
    vtkObjectFactoryCreatevtkTestPoints2);
}

void TestNewPoints(vtkPoints* v, const char* expectedClassName)
{
  if (strcmp(v->GetClassName(), expectedClassName) != 0)
  {
    failed = 1;
    cout << "Test Failed:\nExpected classname: " << expectedClassName
         << "\nCreated classname: " << v->GetClassName() << endl;
  }
}

int TestObjectFactory(int, char*[])
{
  vtkOutputWindow::GetInstance()->PromptUserOff();
  vtkGenericWarningMacro("Test Generic Warning");
  TestFactory* factory = TestFactory::New();
  vtkObjectFactory::RegisterFactory(factory);
  factory->Delete();
  vtkPoints* v = vtkPoints::New();
  TestNewPoints(v, "vtkTestPoints");
  v->Delete();

  // disable all vtkPoints creation with the
  factory->Disable("vtkPoints");
  v = vtkPoints::New();
  TestNewPoints(v, "vtkPoints");

  factory->SetEnableFlag(1, "vtkPoints", "vtkTestPoints2");
  v->Delete();
  v = vtkPoints::New();
  TestNewPoints(v, "vtkTestPoints2");

  factory->SetEnableFlag(0, "vtkPoints", "vtkTestPoints2");
  factory->SetEnableFlag(1, "vtkPoints", "vtkTestPoints");
  v->Delete();
  v = vtkPoints::New();
  TestNewPoints(v, "vtkTestPoints");
  v->Delete();
  vtkOverrideInformationCollection* oic = vtkOverrideInformationCollection::New();
  vtkObjectFactory::GetOverrideInformation("vtkPoints", oic);
  vtkOverrideInformation* oi;
  if (oic->GetNumberOfItems() != 2)
  {
    cout << "Incorrect number of overrides for vtkPoints, expected 2, got: "
         << oic->GetNumberOfItems() << "\n";
    failed = 1;
    if (oic->GetNumberOfItems() < 2)
    {
      return 1;
    }
  }
  vtkCollectionSimpleIterator oicit;
  oic->InitTraversal(oicit);
  oi = oic->GetNextOverrideInformation(oicit);
  oi->GetObjectFactory();

  if (strcmp(oi->GetClassOverrideName(), "vtkPoints") != 0)
  {
    cout << "failed: GetClassOverrideName should be vtkPoints, is: " << oi->GetClassOverrideName()
         << "\n";
    failed = 1;
  }
  if (strcmp(oi->GetClassOverrideWithName(), "vtkTestPoints") != 0)
  {
    cout << "failed: GetClassOverrideWithName should be vtkTestPoints, is: "
         << oi->GetClassOverrideWithName() << "\n";
    failed = 1;
  }
  if (strcmp(oi->GetDescription(), "test vertex factory override") != 0)
  {
    cout << "failed: GetClassOverrideWithName should be test vertex factory override, is: "
         << oi->GetDescription() << "\n";
    failed = 1;
  }

  oi = oic->GetNextOverrideInformation(oicit);
  if (strcmp(oi->GetClassOverrideName(), "vtkPoints") != 0)
  {
    cout << "failed: GetClassOverrideName should be vtkPoints, is: " << oi->GetClassOverrideName()
         << "\n";
    failed = 1;
  }
  if (strcmp(oi->GetClassOverrideWithName(), "vtkTestPoints2") != 0)
  {
    cout << "failed: GetClassOverrideWithName should be vtkTestPoints2, is: "
         << oi->GetClassOverrideWithName() << "\n";
    failed = 1;
  }
  if (strcmp(oi->GetDescription(), "test vertex factory override 2") != 0)
  {
    cout << "failed: GetClassOverrideWithName should be test vertex factory override 2, is: "
         << oi->GetDescription() << "\n";
    failed = 1;
  }
  oic->Delete();
  vtkObjectFactory::UnRegisterAllFactories();
  return failed;
}
