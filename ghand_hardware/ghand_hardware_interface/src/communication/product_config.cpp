// SPDX-License-Identifier: Apache-2.0
#include "ghand_hardware_interface/communication/product_config.hpp"
#include "sdk_products.hpp"
namespace ghand_comm {
const ProductConfig* find_product_config(const std::string& name) {
  for (const auto& product : sdk_products) if (product.name == name) return &product;
  return nullptr;
}
}
