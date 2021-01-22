resource "kubernetes_pod" "connector-mqtt" {
  metadata {
    name = "hetida4office-connector-mqtt"
    namespace = kubernetes_namespace.hetida4office.metadata[0].name
    labels = {
      application = "hetida4office"
      service = "connector-mqtt"
    }
  }
  spec {
    container {
      image = "hetida4office/connector-mqtt:0.0.12"
      name  = "connector-mqtt"
      env {
        name = "SPRING_PROFILES_ACTIVE"
        value = "production"
      }
    }
  }
}

resource "kubernetes_pod" "kafka-consumer" {
  metadata {
    name = "hetida4office-kafka-consumer"
    namespace = kubernetes_namespace.hetida4office.metadata[0].name
    labels = {
      application = "hetida4office"
      service = "kafka-consumer"
    }
  }
  spec {
    container {
      image = "hetida4office/kafka-consumer:0.0.12"
      name  = "kafka-consumer"
      env {
        name = "SPRING_PROFILES_ACTIVE"
        value = "production"
      }
    }
  }
}
