/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDataEncoder.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkDataEncoder
 * @brief   class used to compress/encode images using threads.
 *
 * vtkDataEncoder is used to compress and encode images using threads.
 * Multiple images can be pushed into the encoder for compression and encoding.
 * We use a vtkTypeUInt32 as the key to identify different image pipes. The
 * images in each pipe will be processed in parallel threads. The latest
 * compressed and encoded image can be accessed using GetLatestOutput().
 *
 * vtkDataEncoder uses a thread-pool to do the compression and encoding in
 * parallel.  Note that images may not come out of the vtkDataEncoder in the
 * same order as they are pushed in, if an image pushed in at N-th location
 * takes longer to compress and encode than that pushed in at N+1-th location or
 * if it was pushed in before the N-th location was even taken up for encoding
 * by the a thread in the thread pool.
 */

#ifndef vtkDataEncoder_h
#define vtkDataEncoder_h

#include "vtkDeprecation.h" // needed for exports
#include "vtkObject.h"
#include "vtkSmartPointer.h"  // needed for vtkSmartPointer
#include "vtkWebCoreModule.h" // needed for exports
#include <memory>             // for std::unique_ptr

class vtkUnsignedCharArray;
class vtkImageData;

class VTKWEBCORE_EXPORT vtkDataEncoder : public vtkObject
{
public:
  static vtkDataEncoder* New();
  vtkTypeMacro(vtkDataEncoder, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Define the number of worker threads to use. Default is 3.
   * Initialize() needs to be called after changing the thread count.
   */
  void SetMaxThreads(vtkTypeUInt32);
  vtkGetMacro(MaxThreads, vtkTypeUInt32);
  ///@}

  /**
   * Re-initializes the encoder. This will abort any on going encoding threads
   * and clear internal data-structures.
   */
  void Initialize();

  /**
   * Push an image into the encoder. It is not safe to modify the image
   * after this point, including changing the reference counts for it.
   * You may run into thread safety issues. Typically,
   * the caller code will simply release reference to the data and stop using
   * it. vtkDataEncoder takes over the reference for the image and will call
   * vtkObject::UnRegister() on it when it's done.
   * encoding can be set to 0 to skip encoding.
   */
  VTK_DEPRECATED_IN_9_1_0("replaced by Push")
  void PushAndTakeReference(vtkTypeUInt32 key, vtkImageData*& data, int quality, int encoding = 1);

  /**
   * Push an image into the encoder. The data is considered unchanging and thus
   * should not be modified once pushed. Reference count changes are now thread safe
   * and hence callers should ensure they release the reference held, if
   * appropriate.
   */
  void Push(vtkTypeUInt32 key, vtkImageData* data, int quality, int encoding = 1);

  /**
   * Get access to the most-recent fully encoded result corresponding to the
   * given key, if any. This methods returns true if the \c data obtained is the
   * result from the most recent Push() for the key, if any. If this method
   * returns false, it means that there's some image either being processed on
   * pending processing.
   */
  bool GetLatestOutput(vtkTypeUInt32 key, vtkSmartPointer<vtkUnsignedCharArray>& data);

  /**
   * Flushes the encoding pipe and blocks till the most recently pushed image
   * for the particular key has been processed. This call will block. Once this
   * method returns, caller can use GetLatestOutput(key) to access the processed
   * output.
   */
  void Flush(vtkTypeUInt32 key);

  /**
   * Take an image data and synchronously convert it to a base-64 encoded png.
   */
  const char* EncodeAsBase64Png(vtkImageData* img, int compressionLevel = 5);

  /**
   * Take an image data and synchronously convert it to a base-64 encoded jpg.
   */
  const char* EncodeAsBase64Jpg(vtkImageData* img, int quality = 50);

  /**
   * This method will wait for any running thread to terminate.
   */
  void Finalize();

protected:
  vtkDataEncoder();
  ~vtkDataEncoder() override;

  vtkTypeUInt32 MaxThreads;

private:
  vtkDataEncoder(const vtkDataEncoder&) = delete;
  void operator=(const vtkDataEncoder&) = delete;

  class vtkInternals;
  std::unique_ptr<vtkInternals> Internals;
};

#endif
