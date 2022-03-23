// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2017 NXP. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>

#define PHY_CTRL0			0x0
#define PHY_CTRL0_REF_SSP_EN		BIT(2)
#define PHY_CTRL0_FSEL_MASK		GENMASK(10, 5)
#define PHY_CTRL0_FSEL_24M		0x2a
#define PHY_CTRL0_SSC_RANGE_MASK	GENMASK(23, 21)
#define PHY_CTRL0_SSC_RANGE_4003PPM	(0x2 << 21)

#define PHY_CTRL1			0x4
#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_COMMONONN		BIT(1)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_DCDENB		BIT(17)
#define PHY_CTRL1_CHRGSEL		BIT(18)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2			0x8
#define PHY_CTRL2_TXENABLEN0		BIT(8)
#define PHY_CTRL2_OTG_DISABLE		BIT(9)

#define PHY_CTRL6			0x18
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

#define PHY_CTRL5			0x14
#define PHY_CTRL5_DMPWD_OVERRIDE_SEL	BIT(23)
#define PHY_CTRL5_DMPWD_OVERRIDE	BIT(22)
#define PHY_CTRL5_DPPWD_OVERRIDE_SEL	BIT(21)
#define PHY_CTRL5_DPPWD_OVERRIDE	BIT(20)

#define PHY_CTRL6			0x18
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	BIT(29)
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

#define PHY_STS0			0x40
#define PHY_STS0_OTGSESSVLD		BIT(7)
#define PHY_STS0_CHGDET			BIT(4)
#define PHY_STS0_FSVPLUS		BIT(3)
#define PHY_STS0_FSVMINUS		BIT(2)

struct imx8mq_usb_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *base;
	struct regulator *vbus;
	struct extcon_dev *edev;
	struct notifier_block edev_notifier;
	struct work_struct edev_work;
	int chg_type;
};

static int imx8mq_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		   PHY_CTRL1_COMMONONN);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	value &= ~PHY_CTRL0_SSC_RANGE_MASK;
	value |= PHY_CTRL0_SSC_RANGE_4003PPM;
	writel(value, imx_phy->base + PHY_CTRL0);

	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0;
	writel(value, imx_phy->base + PHY_CTRL2);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~PHY_CTRL6_ALT_CLK_SEL;
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, imx_phy->base + PHY_CTRL6);

	return 0;
}

static int imx8mp_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* USB3.0 PHY signal fsel for 24M ref */
	value = readl(imx_phy->base + PHY_CTRL0);
	value &= ~PHY_CTRL0_FSEL_MASK;
	value |= FIELD_PREP(PHY_CTRL0_FSEL_MASK, PHY_CTRL0_FSEL_24M);
	writel(value, imx_phy->base + PHY_CTRL0);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, imx_phy->base + PHY_CTRL6);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	writel(value, imx_phy->base + PHY_CTRL0);

	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	writel(value, imx_phy->base + PHY_CTRL2);

	udelay(10);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	return 0;
}

static int imx8mq_chg_data_contact_det(struct imx8mq_usb_phy *imx_phy)
{
	int i, data_pin_contact_count = 0;
	u32 val;

	/* Set DMPULLDOWN<#> = 1'b1 (to enable RDM_DWN) */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Set DPPULLDOWN<#> = 1'b0 */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Enable Data Contact Detect (DCD) per the USB BC 1.2 */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_DCDENB, imx_phy->base + PHY_CTRL1);

	for (i = 0; i < 100; i = i + 1) {
		val = readl(imx_phy->base + PHY_STS0);
		/* DP is low */
		if (!(val & PHY_STS0_FSVPLUS)) {
			if (data_pin_contact_count++ > 5)
				/* Data pin makes contact */
				break;
			usleep_range(5000, 10000);
		} else {
			data_pin_contact_count = 0;
			usleep_range(5000, 6000);
		}
	}

	/* Disable DCD after finished data contact check */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_DCDENB;
	writel(val, imx_phy->base + PHY_CTRL1);

	if (i == 100) {
		dev_dbg(&imx_phy->phy->dev,
			"VBUS is coming from a dedicated power supply.\n");
		imx_phy->chg_type = EXTCON_CHG_USB_SLOW;

		/* disable override before finish */
		val = readl(imx_phy->base + PHY_CTRL5);
		val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
		writel(val, imx_phy->base + PHY_CTRL5);

		return -ENXIO;
	}

	/* Set DMPULLDOWN<#> to 1'b0 when DCD is completed */
	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~PHY_CTRL5_DMPWD_OVERRIDE_SEL;
	val |= PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	return 0;
}

static int imx8mq_chg_primary_detect(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDP_SRC is connected to D+ and IDM_SINK is connected to D- */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_CHRGSEL;
	val |= PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0;
	writel(val, imx_phy->base + PHY_CTRL1);

	usleep_range(1000, 2000);

	/* Check if D- is less than VDAT_REF to determine an SDP per BC 1.2 */
	val = readl(imx_phy->base + PHY_STS0);
	if (!(val & PHY_STS0_CHGDET)) {
		dev_dbg(&imx_phy->phy->dev, "It is a SDP.\n");
		imx_phy->chg_type = EXTCON_CHG_USB_SDP;
	}

	return 0;
}

static int imx8mq_phy_chg_secondary_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDM_SRC is connected to D- and IDP_SINK is connected to D+ */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		PHY_CTRL1_CHRGSEL, imx_phy->base + PHY_CTRL1);

	usleep_range(1000, 2000);

	/*
	 * Per BC 1.2, check voltage of D+:
	 * DCP: if greater than VDAT_REF;
	 * CDP: if less than VDAT0_REF.
	 */
	val = readl(imx_phy->base + PHY_STS0);
	if (val & PHY_STS0_CHGDET) {
		dev_dbg(&imx_phy->phy->dev, "It is a DCP.\n");
		imx_phy->chg_type = EXTCON_CHG_USB_DCP;
	} else {
		dev_dbg(&imx_phy->phy->dev, "It is a CDP.\n");
		imx_phy->chg_type = EXTCON_CHG_USB_CDP;
	}

	return 0;
}

static void imx8mq_phy_disable_chg_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
	writel(val, imx_phy->base + PHY_CTRL5);

	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~(PHY_CTRL1_DCDENB | PHY_CTRL1_VDATSRCENB0 |
		 PHY_CTRL1_VDATDETENB0 | PHY_CTRL1_CHRGSEL);
	writel(val, imx_phy->base + PHY_CTRL1);
}

static int imx8mq_phy_charger_detect(struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &imx_phy->phy->dev;
	struct device_node *np = dev->parent->of_node;
	u32 value;
	int ret = 0;

	if (!np)
		return 0;

	if (!imx_phy->edev)
		return 0;

	if (!extcon_get_state(imx_phy->edev, EXTCON_USB))
		return 0;

	if (imx_phy->chg_type != EXTCON_NONE)
		return 0;

	/* Check if vbus is valid */
	value = readl(imx_phy->base + PHY_STS0);
	if (!(value & PHY_STS0_OTGSESSVLD)) {
		dev_err(&imx_phy->phy->dev, "vbus is error\n");
		return -EINVAL;
	}

	ret = imx8mq_chg_data_contact_det(imx_phy);
	if (ret)
		goto out;

	ret = imx8mq_chg_primary_detect(imx_phy);
	if (!ret && imx_phy->chg_type != EXTCON_CHG_USB_SDP)
		ret = imx8mq_phy_chg_secondary_det(imx_phy);

	imx8mq_phy_disable_chg_det(imx_phy);

out:
	extcon_set_state_sync(imx_phy->edev, imx_phy->chg_type, true);

	return ret;
}

static void imx8mq_phy_extcon_notify_worker(struct work_struct *work)
{
	struct imx8mq_usb_phy *imx_phy =
		container_of(work, struct imx8mq_usb_phy, edev_work);

	if (extcon_get_state(imx_phy->edev, EXTCON_USB))
		return;

	if (extcon_get_state(imx_phy->edev, EXTCON_USB_HOST))
		extcon_set_state_sync(imx_phy->edev, EXTCON_USB_HOST, false);

	if (imx_phy->chg_type != EXTCON_NONE)
		extcon_set_state_sync(imx_phy->edev, imx_phy->chg_type, false);

	imx_phy->chg_type = EXTCON_NONE;
}

static int imx8mq_phy_extcon_notify(struct notifier_block *nb,
				  unsigned long event, void *param)
{
	struct imx8mq_usb_phy *imx_phy =
		container_of(nb, struct imx8mq_usb_phy, edev_notifier);
	schedule_work(&imx_phy->edev_work);
	return NOTIFY_OK;
}

static int imx8mq_phy_set_mode(struct phy *phy, enum phy_mode mode,
			       int submode)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);

	if (imx_phy->edev)
		extcon_set_state_sync(imx_phy->edev, EXTCON_USB_HOST, mode == PHY_MODE_USB_HOST);

	if (mode == PHY_MODE_USB_DEVICE)
		return imx8mq_phy_charger_detect(imx_phy);

	return 0;
}

static int imx8mq_phy_power_on(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;
	int ret;

	ret = regulator_enable(imx_phy->vbus);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx_phy->clk);
	if (ret)
		return ret;

	/* Disable rx term override */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	return 0;
}

static int imx8mq_phy_power_off(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* Override rx term to be 0 */
	value = readl(imx_phy->base + PHY_CTRL6);
	value |= PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	clk_disable_unprepare(imx_phy->clk);
	regulator_disable(imx_phy->vbus);

	return 0;
}

static const struct phy_ops imx8mq_usb_phy_ops = {
	.init		= imx8mq_usb_phy_init,
	.power_on	= imx8mq_phy_power_on,
	.power_off	= imx8mq_phy_power_off,
	.set_mode	= imx8mq_phy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct phy_ops imx8mp_usb_phy_ops = {
	.init		= imx8mp_usb_phy_init,
	.power_on	= imx8mq_phy_power_on,
	.power_off	= imx8mq_phy_power_off,
	.owner		= THIS_MODULE,
};

static const struct of_device_id imx8mq_usb_phy_of_match[] = {
	{.compatible = "fsl,imx8mq-usb-phy",
	 .data = &imx8mq_usb_phy_ops,},
	{.compatible = "fsl,imx8mp-usb-phy",
	 .data = &imx8mp_usb_phy_ops,},
	{ }
};
MODULE_DEVICE_TABLE(of, imx8mq_usb_phy_of_match);

static const unsigned int imx8mq_usb_phy_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_DCP,
	EXTCON_CHG_USB_CDP,
	EXTCON_CHG_USB_SLOW,
	EXTCON_NONE,
};

static int imx8mq_usb_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct imx8mq_usb_phy *imx_phy;
	const struct phy_ops *phy_ops;
	int ret;

	imx_phy = devm_kzalloc(dev, sizeof(*imx_phy), GFP_KERNEL);
	if (!imx_phy)
		return -ENOMEM;

	imx_phy->clk = devm_clk_get(dev, "phy");
	if (IS_ERR(imx_phy->clk)) {
		dev_err(dev, "failed to get imx8mq usb phy clock\n");
		return PTR_ERR(imx_phy->clk);
	}

	imx_phy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(imx_phy->base))
		return PTR_ERR(imx_phy->base);

	phy_ops = of_device_get_match_data(dev);
	if (!phy_ops)
		return -EINVAL;

	imx_phy->phy = devm_phy_create(dev, NULL, phy_ops);
	if (IS_ERR(imx_phy->phy))
		return PTR_ERR(imx_phy->phy);

	imx_phy->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(imx_phy->vbus))
		return PTR_ERR(imx_phy->vbus);

	phy_set_drvdata(imx_phy->phy, imx_phy);
	platform_set_drvdata(pdev, imx_phy);

	imx_phy->edev = devm_extcon_dev_allocate(dev, imx8mq_usb_phy_extcon_cable);
	if (IS_ERR(imx_phy->edev)) {
		dev_err(dev, "failed to allocate memory for extcon\n");
		imx_phy->edev = NULL;
	}

	if (imx_phy->edev) {
		/* Register extcon device */
		ret = devm_extcon_dev_register(dev, imx_phy->edev);
		if (ret)
			dev_err(dev, "failed to register extcon device: %d\n", ret);

		/* set initial state */
		extcon_set_state_sync(imx_phy->edev, EXTCON_USB, false);
		extcon_set_state_sync(imx_phy->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(imx_phy->edev, EXTCON_CHG_USB_SDP, false);
		extcon_set_state_sync(imx_phy->edev, EXTCON_CHG_USB_DCP, false);
		extcon_set_state_sync(imx_phy->edev, EXTCON_CHG_USB_CDP, false);
		extcon_set_state_sync(imx_phy->edev, EXTCON_CHG_USB_SLOW, false);
		imx_phy->chg_type = EXTCON_NONE;

		INIT_WORK(&imx_phy->edev_work, imx8mq_phy_extcon_notify_worker);
		imx_phy->edev_notifier.notifier_call = imx8mq_phy_extcon_notify;

		ret = devm_extcon_register_notifier_all(dev, imx_phy->edev,
							&imx_phy->edev_notifier);
		if (ret)
			dev_err(dev, "register extcon notifier failed\n");
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver imx8mq_usb_phy_driver = {
	.probe	= imx8mq_usb_phy_probe,
	.driver = {
		.name	= "imx8mq-usb-phy",
		.of_match_table	= imx8mq_usb_phy_of_match,
	}
};
module_platform_driver(imx8mq_usb_phy_driver);

MODULE_DESCRIPTION("FSL IMX8MQ USB PHY driver");
MODULE_LICENSE("GPL");
