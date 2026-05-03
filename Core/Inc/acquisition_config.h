#ifndef ACQUISITION_CONFIG_H
#define ACQUISITION_CONFIG_H

#define ACQ_MODE_POLLING 0
#define ACQ_MODE_IT      1
#define ACQ_MODE_DMA     2

/* Select one implementation, then rebuild and upload the firmware. */
#ifndef ACQ_MODE
#define ACQ_MODE ACQ_MODE_DMA
#endif

#if ACQ_MODE != ACQ_MODE_POLLING && ACQ_MODE != ACQ_MODE_IT && ACQ_MODE != ACQ_MODE_DMA
#error "Invalid ACQ_MODE. Select ACQ_MODE_POLLING, ACQ_MODE_IT or ACQ_MODE_DMA."
#endif

#endif
