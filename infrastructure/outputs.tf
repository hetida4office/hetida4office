output "resource_group_name" {
  value = azurerm_resource_group.h4o.name
}

output "kubernetes_cluster_name" {
  value = azurerm_kubernetes_cluster.h4o.name
}

output "host" {
  value = azurerm_kubernetes_cluster.h4o.kube_config.0.host
}

output "client_key" {
  value = azurerm_kubernetes_cluster.h4o.kube_config.0.client_key
}

output "client_certificate" {
  value = azurerm_kubernetes_cluster.h4o.kube_config.0.client_certificate
}

output "kube_config" {
  value = azurerm_kubernetes_cluster.h4o.kube_config_raw
}

output "cluster_username" {
  value = azurerm_kubernetes_cluster.h4o.kube_config.0.username
}

output "cluster_password" {
  value = azurerm_kubernetes_cluster.h4o.kube_config.0.password
}
