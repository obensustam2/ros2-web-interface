import { createI18n } from 'vue-i18n';
import de from './de.json';
import en from './en.json';

// Dynamically import the language files based on the locale
const i18n = createI18n({
  legacy: false,  // Enable Composition API mode
  locale: 'en',   // Default locale
  messages: {
    de,
    en,
  },
});

export default i18n;

