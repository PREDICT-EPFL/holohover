/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ["./configserver.html"],
  theme: {
    extend: {},
  },
  plugins: [
    require('@tailwindcss/forms'),
  ],
}
