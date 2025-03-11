#include <linux/of.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>

/* Log macros */
#define fstabdt_err(fmt, ...) pr_err("fstabdt_disable: " fmt, ##__VA_ARGS__)
#define fstabdt_info(fmt, ...) pr_info("fstabdt_disable: " fmt, ##__VA_ARGS__)

static bool delete_fstabdt = true; /* Default is true unless fstabdt_keep is present */
module_param(delete_fstabdt, bool, 0444);

static int __init modify_fstab_entry(void)
{
	struct device_node *fstab_node = NULL, *vendor_node = NULL;
	struct property *prop = NULL;
	int ret = 0;

	/* Parse the kernel command line for "fstabdt_keep" */
	if (strstr(saved_command_line, "fstabdt_keep"))
		delete_fstabdt = false;

	if (!delete_fstabdt) {
		fstabdt_info("fstabdt_keep is present. Skipping modification.\n");
		goto out;
	}

	fstab_node = of_find_node_by_path("/firmware/android/fstab");
	if (!fstab_node) {
		fstabdt_err("Failed to find fstab node\n");
		ret = -ENODEV;
		goto out;
	}

	vendor_node = of_get_child_by_name(fstab_node, "vendor");
	if (!vendor_node) {
		fstabdt_err("Failed to find vendor node\n");
		ret = -ENODEV;
		goto out_fstab_node;
	}

	/* Find the status property */
	prop = of_find_property(vendor_node, "status", NULL);
	if (!prop) {
		fstabdt_err("Failed to find status property\n");
		ret = -ENODEV;
		goto out_vendor_node;
	}

	/* Remove the current status property */
	of_remove_property(vendor_node, prop);

	/* Allocate and initialize a new property with the value "disabled" */
	prop = kzalloc(sizeof(*prop), GFP_KERNEL);
	if (!prop) {
		fstabdt_err("Failed to allocate memory for property\n");
		ret = -ENOMEM;
		goto out_vendor_node;
	}
	prop->name = "status";
	prop->length = sizeof("disabled");
	prop->value = kstrdup("disabled", GFP_KERNEL);
	if (!prop->value) {
		fstabdt_err("Failed to allocate memory for property value\n");
		kfree(prop);
		ret = -ENOMEM;
		goto out_vendor_node;
	}

	ret = of_add_property(vendor_node, prop);
	if (ret) {
		fstabdt_err("Failed to add new status property\n");
		kfree(prop->value);
		kfree(prop);
		goto out_vendor_node;
	}

	fstabdt_info("fstab entry modified successfully\n");

out_vendor_node:
	if (vendor_node)
		of_node_put(vendor_node);
out_fstab_node:
	if (fstab_node)
		of_node_put(fstab_node);
out:
	return ret;
}

early_initcall(modify_fstab_entry);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Flopster101");
MODULE_DESCRIPTION("Small driver to disable the device-tree fstab for two-stage init ROMs.");
