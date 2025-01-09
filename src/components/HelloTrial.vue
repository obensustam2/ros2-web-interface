<template>
  <div id="app">
    <!-- Header -->
    <header class="header">
      <h1>ROS Smarthand Project</h1>
    </header>

    <div class="layout">
      <!-- Sidebar -->
      <div class="sidebar">
        <img src="/home/smarthand/ros2-web-interface/public/logostreicher.png" alt="Logo" class="sidebar-logo" />
        <h2>Sidebar</h2>
        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_eye"></svg-icon>
        </button>

        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_joystick"></svg-icon>
        </button>
        
        <button class="sidebar-button">MOVE TO</button>

        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_workflow"></svg-icon>
        </button>

        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_admin"></svg-icon>
        </button>

        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_gear"></svg-icon>
        </button>

        <button class="sidebar-button">
          <svg-icon type="mdi" :path="path_web"></svg-icon>
        </button>

        <div class="sidebar-bottom-button-wrapper">
          <svg-icon type="mdi" :path="path_log"></svg-icon>
        </div>

      </div>

      <!-- Main Content -->
      <div class="main-content">
        <h1>Main Content</h1>
        <p>This section spans the full height of the page.</p>
      </div>

      <!-- Column 3: Two Rows -->
      <div class="additional-content">
        <!-- Row 1 -->
        <div class="content-2">
          <h2>Content 2</h2>
          <p>This is the first row in column 3.</p>
        </div>
        <!-- Row 2 -->
        <div class="content-3">
          <h2>Content 3</h2>
          <p>This is the second row in column 3.</p>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import SvgIcon from '@jamescoyle/vue-icon';
import { mdiWeb, mdiCogOutline, mdiEyeOutline, mdiAccountCircle, mdiController, 
          mdiClipboardFlow, mdiFileArrowLeftRightOutline } from '@mdi/js';

export default {
  name: "TrialComponent", // Fixed component name to be multi-word

  components: {
    SvgIcon
  },

  data() {
    return {
      path_web: mdiWeb,
      path_gear: mdiCogOutline,
      path_admin: mdiAccountCircle,
      path_eye: mdiEyeOutline,
      path_joystick: mdiController,
      path_workflow: mdiClipboardFlow,
      path_log: mdiFileArrowLeftRightOutline
    };
  }
};
</script>

<style scoped>
/* General styling */
body {
  margin: 0;
  font-family: Arial, sans-serif;
}

/* Header styling */
.header {
  background-color: #1976d2;
  color: white;
  padding: 15px 20px;
  text-align: center;
  font-size: 24px;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.2);
}

/* Layout styling */
.layout {
  display: grid;
  grid-template-columns: 1fr 2fr 1fr; /* Sidebar, Main Content, Column 3 */
  grid-template-rows: auto; /* Single row layout for columns */
  height: calc(100vh - 70px); /* Subtract header height */
}

/* Sidebar styling */
.sidebar {
  background-color: #333;
  color: white;
  padding: 20px;
  box-sizing: border-box;
  display: flex;
  flex-direction: column;
  gap: 10px; /* Spacing between buttons */
}

/* Sidebar buttons styling */
.sidebar-button {
  background-color: #555;
  color: white;
  border: none;
  padding: 10px 15px;
  text-align: center;
  font-size: 16px;
  cursor: pointer;
  border-radius: 5px;
  transition: background-color 0.3s;
}

.sidebar-button:hover {
  background-color: #777;
}

/* Sidebar bottom button wrapper */
.sidebar-bottom-button-wrapper {
  margin-top: auto;
}

/* Sidebar logo styling */
.sidebar-logo {
  width: 100%;
  margin-bottom: 20px;
}

/* Main content styling */
.main-content {
  background-color: #f4f4f4;
  padding: 20px;
  box-sizing: border-box;
}

/* Additional content styling (Column 3) */
.additional-content {
  display: grid;
  grid-template-rows: 1fr 1fr; /* Two rows sharing equal height */
  background-color: #e8eaf6;
}

/* Content 2 styling (Row 1) */
.content-2 {
  background-color: #e0f7fa;
  padding: 20px;
  box-sizing: border-box;
}

/* Content 3 styling (Row 2) */
.content-3 {
  background-color: #e1bee7;
  padding: 20px;
  box-sizing: border-box;
}
</style>
