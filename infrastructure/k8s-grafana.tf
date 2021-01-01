resource "kubernetes_pod" "grafana" {
  metadata {
    name = "hetida4office-grafana"
    namespace = kubernetes_namespace.hetida4office.metadata[0].name
    labels = {
      application = "hetida4office"
      service = "grafana"
    }
  }
  spec {
    security_context {
        fs_group = 0
    }
    container {
      image = "grafana/grafana"
      name  = "grafana"
      port {
        container_port = 3000
      }
      volume_mount {
        name = "grafana-data"
        mount_path = "/var/lib/grafana"
        sub_path = "grafana"
      }
    }
    volume {
      name = "grafana-data"
      azure_disk {
          caching_mode = "None"
          kind = "Managed"
          disk_name = "hetida4office"
          data_disk_uri = azurerm_managed_disk.h4o.id
      }
    }
  }
}

resource "kubernetes_service" "grafana" {
  metadata {
    name = "hetida4office-grafana"
    namespace = kubernetes_namespace.hetida4office.metadata[0].name
    labels = {
        application = "hetida4office"
        service = "grafana"
    }
  }
  spec {
    selector = {
      service = "grafana"
    }
    port {
      port        = 80
      target_port = 3000
    }
    type = "LoadBalancer"
  }
}
