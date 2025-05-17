import { createApp } from 'vue'
import App from './App.vue'
// import i18n from './i18n/i18n';  // Import i18n configuration
import vuetify from './plugins/vuetify'
import { loadFonts } from './plugins/webfontloader'

loadFonts()

createApp(App)
  .use(vuetify)
  .mount('#app')
  // .use(i18n)  // Use i18n in your app

