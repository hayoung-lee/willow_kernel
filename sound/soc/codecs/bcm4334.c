/*
 * bcm4334.c
 *
 * test version
 */

#include <linux/init.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define BCM4334_RATES SNDRV_PCM_RATE_8000_96000

#define BCM4334_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

//static int bcm4334_hw_params(struct snd_pcm_substream *substream,
//			    struct snd_pcm_hw_params *params,
//			    struct snd_soc_dai *dai)
//{
//	return 0;
//}
//
//static struct snd_soc_dai_ops bcm4334_aif1_dai_ops = {
//	.hw_params	= bcm4334_hw_params,
//};

static struct snd_soc_dai_driver bcm4334_dai = {
	.name = "bcm4334-pcm",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM4334_RATES,
		.formats = BCM4334_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = BCM4334_RATES,
		.formats = BCM4334_FORMATS,
	},
	//.ops = &bcm4334_aif1_dai_ops,
	.symmetric_rates = 1
};

static int bcm4334_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int  bcm4334_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_bcm4334 = {
	.probe =  bcm4334_codec_probe,
	.remove = bcm4334_codec_remove,
};

static struct device_driver bcm4334_driver = {
	.name = "bcm4334-pcm",
};

static struct device bcm4334_device = {
	.init_name = "bcm4334-pcm",
	.driver = &bcm4334_driver,
};

static __init int bcm4334_init(void)
{
	int ret;

	ret = device_register(&bcm4334_device);
	if (ret < 0)
		return ret;

	ret = snd_soc_register_codec(&bcm4334_device, &soc_codec_dev_bcm4334,
			&bcm4334_dai, 1);

	return ret;
}
module_init(bcm4334_init);

static __exit void bcm4334_exit(void)
{
	snd_soc_unregister_codec(&bcm4334_device);
	device_unregister(&bcm4334_device);
}
module_exit(bcm4334_exit);

MODULE_DESCRIPTION("BCM4334 PCM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm4334-pcm");
