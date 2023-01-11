# dt_ood_cps

## stream_to_dataset

Utility node which subscribes to a topic publishing `DetectorInput` messages, cuts out each detected segment from the image and saves it into a directory, creating an image dataset.

Each image is named as {frame number}_{segment number}. For each frame, if there are N number of segments, then there will be N number of images created.

A random directory will be created in the `export_bin` directory to save the created images.

It may be necessary to reduce the publishing rate of the `~/ood_input` to dropped lost frames.

### Launch configurations

```txt
1. export_bin

Full path to the directory to export created images to. A new directory will be created in `export_bin`.

2. crop_type

* passthrough - Size of output images is the same as the subscribed image. Segment is extracted as is. 

* fit - Size of output images fits around the segment, hence the image size depends on the segment size. A mask is applied onto the image so that only the segment is extracted, hence background of the created image is black. Segment is extracted as it.

* bin - Size of output images fits around the segment, hence depends on the segment size. A mask is applied onto the image so that only the segment is extracted, hence background of the created image is black. Segment is extracted as is. The image is then resized to the closest bin.

* trim - Size of the output image is fixed. If the segment is larger than can fit inside the image, its edges are cropped/removed/trimmed off.
```
