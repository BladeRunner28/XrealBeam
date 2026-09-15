plugins {
    id("com.android.application")
}

android {
    namespace = "com.xman.xrealbeam"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.xman.xrealbeam"
        minSdk = 29          // Note 20 Ultra shipped on Android 10 (API 29)
        targetSdk = 34       // the device is on Android 13 (API 33)
        versionCode = 7
        versionName = "0.5.1-cinema-headlocked"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    buildTypes {
        getByName("debug") {
            isMinifyEnabled = false
        }
    }
}
